import sys
import time
import math
import html
import threading

import spidev
from gpiozero import DigitalOutputDevice

from PyQt6.QtCore import (
    Qt,
    QThread,
    QTimer,
    pyqtSignal
)

from PyQt6.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QFrame,
    QTextEdit,
    QMessageBox
)

# Offline PMTiles map (must be imported before QApplication is created)
from offline_map import OfflineMapWidget


# ============================================================
# RASPBERRY PI 5 + RA-02 SX1278
# TWO-WAY LORA COMMUNICATION + ESP32 GPS
# PYQT6 DASHBOARD
# ============================================================


# ============================================================
# GPIO  (UNCHANGED)
# ============================================================

# Physical Pin 15 = GPIO22
RESET_PIN = 22


# ============================================================
# SPI  (UNCHANGED)
# ============================================================

SPI_BUS = 0
SPI_DEVICE = 0

# Jumper wires often fail at 2 MHz and the chip then reads
# back 0x00 / 0xFF instead of 0x12.
SPI_SPEED_HZ = 500000


# ============================================================
# BASE STATION (THIS RASPBERRY PI)
# ============================================================

BASE_NAME = "Base"

BASE_LATITUDE = 6.9126422
BASE_LONGITUDE = 79.8509647

# The node sends a GPS beacon every 5 s. If nothing at all is heard
# from it for this long, the dashboard warns it may be out of range.
NODE_TIMEOUT_S = 20

# Shown until the node tells us its real name
DEFAULT_NODE_NAME = "Node"


# ============================================================
# PACKET FORMATS (NODE -> BASE)
# ============================================================
#
#   MSG|<node name>|<lat>|<lon>|<fix>|<fix age s>|<message>
#
#       fix = LIVE  -> node has a GPS fix right now
#       fix = LAST  -> node lost GPS, lat/lon is its LAST KNOWN
#                      position, fix age = seconds since that fix
#       fix = NONE  -> node has never had a fix (lat/lon empty)
#
#   GPS|<node name>|<lat>|<lon>|<fix>|<fix age s>
#       GPS beacon, sent by the node every few seconds on its own
#       (same fix values as above), so the location keeps updating
#       even when nobody is typing messages
#
#   NAME|<node name>
#       sent when the name is saved in Node Settings
#
# The old format  MSG|lat|lon|message  /  MSG|NO_GPS||message
# is still understood, in case an old node firmware is running.
#
# BASE -> NODE messages are plain text (unchanged).
# ============================================================


# ============================================================
# SX1278 REGISTERS
# ============================================================

REG_FIFO                    = 0x00
REG_OP_MODE                 = 0x01
REG_FRF_MSB                 = 0x06
REG_FRF_MID                 = 0x07
REG_FRF_LSB                 = 0x08
REG_PA_CONFIG               = 0x09
REG_LNA                     = 0x0C
REG_FIFO_ADDR_PTR           = 0x0D
REG_FIFO_TX_BASE_ADDR       = 0x0E
REG_FIFO_RX_BASE_ADDR       = 0x0F
REG_FIFO_RX_CURRENT_ADDR    = 0x10
REG_IRQ_FLAGS               = 0x12
REG_RX_NB_BYTES             = 0x13
REG_PKT_SNR_VALUE           = 0x19
REG_PKT_RSSI_VALUE          = 0x1A
REG_MODEM_CONFIG1           = 0x1D
REG_MODEM_CONFIG2           = 0x1E
REG_PREAMBLE_MSB            = 0x20
REG_PREAMBLE_LSB            = 0x21
REG_PAYLOAD_LENGTH          = 0x22
REG_MODEM_CONFIG3           = 0x26
REG_SYNC_WORD               = 0x39
REG_VERSION                 = 0x42


# ============================================================
# SX1278 MODES
# ============================================================

LONG_RANGE_MODE = 0x80

MODE_SLEEP = 0x00
MODE_STDBY = 0x01
MODE_TX = 0x03
MODE_RX_CONT = 0x05


# ============================================================
# IRQ FLAGS
# ============================================================

IRQ_RX_DONE = 0x40
IRQ_PAYLOAD_CRC_ERROR = 0x20
IRQ_TX_DONE = 0x08


# ============================================================
# HELPERS
# ============================================================

def format_age(seconds):

    s = int(max(0, seconds))

    if s < 60:
        return "{}s".format(s)

    m, s = divmod(s, 60)

    if m < 60:
        return "{}m {}s".format(m, s)

    h, m = divmod(m, 60)

    if h < 24:
        return "{}h {}m".format(h, m)

    d, h = divmod(h, 24)

    return "{}d {}h".format(d, h)


def distance_km(lat1, lon1, lat2, lon2):

    # Haversine great-circle distance
    r = 6371.0

    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)

    a = (
        math.sin(dp / 2) ** 2
        + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    )

    return 2 * r * math.asin(math.sqrt(a))


# ============================================================
# LORA CLASS
# ============================================================

class LoRaRadio:

    def __init__(self):

        self.reset = None
        self.spi = None

        self.lock = threading.Lock()

        self.initialized = False


    # ========================================================
    # SETUP GPIO AND SPI
    # ========================================================

    def setup(self):

        self.reset = DigitalOutputDevice(
            RESET_PIN,
            active_high=True,
            initial_value=True
        )

        self.spi = spidev.SpiDev()

        self.spi.open(
            SPI_BUS,
            SPI_DEVICE
        )

        self.spi.max_speed_hz = SPI_SPEED_HZ
        self.spi.mode = 0
        self.spi.lsbfirst = False


    # ========================================================
    # REGISTER FUNCTIONS
    # ========================================================

    def write_register(self, address, value):

        self.spi.xfer2([
            address | 0x80,
            value & 0xFF
        ])


    def read_register(self, address):

        response = self.spi.xfer2([
            address & 0x7F,
            0x00
        ])

        return response[1]


    # ========================================================
    # FIFO FUNCTIONS
    # ========================================================

    def write_fifo(self, data):

        self.spi.xfer2(
            [REG_FIFO | 0x80] + list(data)
        )


    def read_fifo(self, length):

        response = self.spi.xfer2(
            [REG_FIFO & 0x7F] + [0x00] * length
        )

        return response[1:]


    # ========================================================
    # RESET RA-02
    # ========================================================

    def reset_lora(self):

        self.reset.off()

        time.sleep(0.1)

        self.reset.on()

        # Give the chip time to come out of reset
        # before the first SPI read.
        time.sleep(0.2)


    # ========================================================
    # SET FREQUENCY
    # ========================================================

    def set_frequency(self):

        frequency = 433000000

        frf = int(
            (frequency * (1 << 19)) / 32000000
        )

        self.write_register(REG_FRF_MSB, (frf >> 16) & 0xFF)
        self.write_register(REG_FRF_MID, (frf >> 8) & 0xFF)
        self.write_register(REG_FRF_LSB, frf & 0xFF)


    # ========================================================
    # START RECEIVE
    # ========================================================

    def start_receive(self):

        self.write_register(REG_IRQ_FLAGS, 0xFF)

        rx_base = self.read_register(REG_FIFO_RX_BASE_ADDR)

        self.write_register(REG_FIFO_ADDR_PTR, rx_base)

        self.write_register(
            REG_OP_MODE,
            LONG_RANGE_MODE | MODE_RX_CONT
        )


    # ========================================================
    # INITIALIZE RADIO
    # ========================================================

    def initialize(self):

        version = 0

        for attempt in range(5):

            self.reset_lora()

            version = self.read_register(REG_VERSION)

            print(
                "SX1278 Version (try {}): 0x{:02X}".format(
                    attempt + 1,
                    version
                )
            )

            if version == 0x12:
                break

            time.sleep(0.2)


        if version != 0x12:

            raise RuntimeError(
                "SX1278 NOT DETECTED. "
                "Expected 0x12, received 0x{:02X}. "
                "(0x00 = MISO low / no response, "
                "0xFF = MISO high / chip in reset or no power)".format(
                    version
                )
            )


        # SLEEP
        self.write_register(REG_OP_MODE, LONG_RANGE_MODE | MODE_SLEEP)
        time.sleep(0.01)

        # STANDBY
        self.write_register(REG_OP_MODE, LONG_RANGE_MODE | MODE_STDBY)

        # FREQUENCY
        self.set_frequency()

        # POWER
        self.write_register(REG_PA_CONFIG, 0x8F)

        # LNA
        self.write_register(REG_LNA, 0x23)

        # FIFO
        self.write_register(REG_FIFO_TX_BASE_ADDR, 0x00)
        self.write_register(REG_FIFO_RX_BASE_ADDR, 0x00)

        # BW = 125 kHz, Coding Rate = 4/5, Explicit Header
        self.write_register(REG_MODEM_CONFIG1, 0x72)

        # SF7, CRC ON
        self.write_register(REG_MODEM_CONFIG2, 0x74)

        # AGC ON
        self.write_register(REG_MODEM_CONFIG3, 0x04)

        # PREAMBLE = 8
        self.write_register(REG_PREAMBLE_MSB, 0x00)
        self.write_register(REG_PREAMBLE_LSB, 0x08)

        # SYNC WORD
        self.write_register(REG_SYNC_WORD, 0x12)

        # CLEAR IRQ
        self.write_register(REG_IRQ_FLAGS, 0xFF)

        # START RECEIVE
        self.start_receive()

        self.initialized = True


    # ========================================================
    # SEND MESSAGE
    # ========================================================

    def send_message(self, message):

        if not message:
            return False

        data = message.encode("utf-8")

        if len(data) > 240:

            raise ValueError(
                "Message is too long. Maximum is 240 bytes."
            )


        with self.lock:

            # STANDBY
            self.write_register(REG_OP_MODE, LONG_RANGE_MODE | MODE_STDBY)
            time.sleep(0.01)

            # CLEAR IRQ
            self.write_register(REG_IRQ_FLAGS, 0xFF)

            # FIFO POINTER -> TX BASE
            tx_base = self.read_register(REG_FIFO_TX_BASE_ADDR)
            self.write_register(REG_FIFO_ADDR_PTR, tx_base)

            # WRITE MESSAGE
            self.write_fifo(data)

            # PAYLOAD LENGTH
            self.write_register(REG_PAYLOAD_LENGTH, len(data))

            # START TX
            self.write_register(REG_OP_MODE, LONG_RANGE_MODE | MODE_TX)

            start_time = time.time()

            while True:

                irq = self.read_register(REG_IRQ_FLAGS)

                if irq & IRQ_TX_DONE:
                    break

                if time.time() - start_time > 5:

                    self.start_receive()

                    raise TimeoutError(
                        "LoRa transmission timed out."
                    )

                time.sleep(0.002)

            # CLEAR TX DONE
            self.write_register(REG_IRQ_FLAGS, IRQ_TX_DONE)

            # RETURN TO RECEIVE MODE
            self.start_receive()


        return True


    # ========================================================
    # RECEIVE MESSAGE
    # ========================================================

    def receive_message(self):

        with self.lock:

            irq = self.read_register(REG_IRQ_FLAGS)

            # NO PACKET
            if not (irq & IRQ_RX_DONE):
                return None

            # CRC ERROR
            if irq & IRQ_PAYLOAD_CRC_ERROR:

                self.write_register(REG_IRQ_FLAGS, 0xFF)

                self.start_receive()

                return {
                    "type": "CRC_ERROR"
                }

            # PACKET LENGTH + LOCATION
            length = self.read_register(REG_RX_NB_BYTES)

            current_address = self.read_register(
                REG_FIFO_RX_CURRENT_ADDR
            )

            self.write_register(REG_FIFO_ADDR_PTR, current_address)

            # READ PACKET
            data = self.read_fifo(length)

            # RSSI (offset 164 for the 433 MHz band)
            rssi = self.read_register(REG_PKT_RSSI_VALUE) - 164

            # SNR
            snr_raw = self.read_register(REG_PKT_SNR_VALUE)

            if snr_raw > 127:
                snr_raw -= 256

            snr = snr_raw / 4.0

            # CLEAR IRQ + BACK TO RECEIVE
            self.write_register(REG_IRQ_FLAGS, 0xFF)

            self.start_receive()


        # Decode outside the lock so TX is not held up
        packet = bytes(data).decode("utf-8", errors="replace")

        return self.parse_packet(packet, rssi, snr)


    # ========================================================
    # PARSE PACKET
    # Checks EVERY packet for a name and GPS coordinates
    # ========================================================

    @staticmethod
    def parse_packet(packet, rssi, snr):

        result = {
            "type": "MESSAGE",
            "message": packet,
            "node": None,
            "latitude": None,
            "longitude": None,
            "fix": "NONE",
            "fix_age": None,
            "rssi": rssi,
            "snr": snr
        }


        # ----------------------------------------------------
        # NAME|<node name>
        # ----------------------------------------------------

        if packet.startswith("NAME|"):

            name = packet[5:].strip()

            if name:

                result["type"] = "NAME"
                result["node"] = name
                result["message"] = ""

            return result


        # ----------------------------------------------------
        # GPS BEACON
        # GPS|node|lat|lon|LIVE/LAST/NONE|age
        # ----------------------------------------------------

        if packet.startswith("GPS|"):

            parts = packet.split("|")

            if len(parts) == 6 and parts[4] in ("LIVE", "LAST", "NONE"):

                result["type"] = "GPS_BEACON"
                result["node"] = parts[1].strip() or None
                result["message"] = ""

                if parts[4] in ("LIVE", "LAST"):

                    try:

                        result["latitude"] = float(parts[2])
                        result["longitude"] = float(parts[3])
                        result["fix"] = parts[4]
                        result["fix_age"] = (
                            int(parts[5]) if parts[5].isdigit() else 0
                        )

                    except ValueError:

                        result["latitude"] = None
                        result["longitude"] = None

                return result


        if not packet.startswith("MSG|"):

            return result


        # ----------------------------------------------------
        # NEW FORMAT
        # MSG|node|lat|lon|LIVE/LAST/NONE|age|message
        # ----------------------------------------------------

        parts = packet.split("|", 6)

        if len(parts) == 7 and parts[4] in ("LIVE", "LAST", "NONE"):

            result["node"] = parts[1].strip() or None
            result["message"] = parts[6]

            if parts[4] in ("LIVE", "LAST"):

                try:

                    result["latitude"] = float(parts[2])
                    result["longitude"] = float(parts[3])
                    result["fix"] = parts[4]
                    result["fix_age"] = (
                        int(parts[5]) if parts[5].isdigit() else 0
                    )
                    result["type"] = "GPS_MESSAGE"

                except ValueError:

                    result["latitude"] = None
                    result["longitude"] = None

            return result


        # ----------------------------------------------------
        # OLD FORMAT
        # MSG|lat|lon|message  or  MSG|NO_GPS||message
        # ----------------------------------------------------

        parts = packet.split("|", 3)

        if len(parts) == 4:

            result["message"] = parts[3]

            if parts[1] != "NO_GPS":

                try:

                    result["latitude"] = float(parts[1])
                    result["longitude"] = float(parts[2])
                    result["fix"] = "LIVE"
                    result["fix_age"] = 0
                    result["type"] = "GPS_MESSAGE"

                except ValueError:

                    result["message"] = packet

        return result


    # ========================================================
    # CLOSE RADIO
    # ========================================================

    def close(self):

        try:

            if self.spi is not None:

                try:

                    self.write_register(
                        REG_OP_MODE,
                        LONG_RANGE_MODE | MODE_SLEEP
                    )

                except Exception:

                    pass

                self.spi.close()

        except Exception:

            pass


        try:

            if self.reset is not None:

                self.reset.close()

        except Exception:

            pass


        self.initialized = False


# ============================================================
# RECEIVE THREAD
# ============================================================

class ReceiveThread(QThread):

    message_received = pyqtSignal(dict)

    error_occurred = pyqtSignal(str)


    def __init__(self, radio):

        super().__init__()

        self.radio = radio

        self.running = True


    def run(self):

        while self.running:

            try:

                packet = self.radio.receive_message()

                if packet is not None:

                    self.message_received.emit(packet)

            except Exception as e:

                self.error_occurred.emit(str(e))

                time.sleep(0.5)

            time.sleep(0.01)


    def stop(self):

        self.running = False

        self.wait(1000)


# ============================================================
# MAIN DASHBOARD
# ============================================================

class LoRaDashboard(QMainWindow):

    # Signals for the send thread
    send_success_signal = pyqtSignal(str)
    send_error_signal = pyqtSignal(str)


    def __init__(self):

        super().__init__()


        self.radio = LoRaRadio()

        self.receive_thread = None


        # ----------------------------------------------------
        # NODE + GPS STATE
        # ----------------------------------------------------

        # Name of the node, taken from its packets
        self.node_name = DEFAULT_NODE_NAME

        # LAST KNOWN location of the node. Updated every time a
        # packet with coordinates arrives and NEVER cleared, so
        # if the node loses GPS we still know where it was.
        #   {"latitude", "longitude", "fix_time", "fix"}
        self.last_known = None

        # Current GPS state reported by the node: LIVE / LAST / NONE
        # (None until the first packet arrives)
        self.node_fix = None

        # When ANY packet was last heard from the node, and its RSSI.
        # Lets us tell "node lost GPS" apart from "node out of range".
        self.last_heard = None
        self.last_rssi = None
        self.link_lost = False


        self.setWindowTitle(
            "ResQNet - LoRa Communication Dashboard"
        )

        self.setMinimumSize(1200, 800)


        self.create_ui()


        # Radio setup runs on the main thread after the window
        # is shown, so the error dialog can appear properly.
        QTimer.singleShot(200, self.start_initialization)


        # Put the Base on the map once the map has had time to load
        QTimer.singleShot(1500, self.show_base_on_map)


        # Re-check the GPS state every second (keeps fix age live)
        self.gps_timer = QTimer(self)
        self.gps_timer.timeout.connect(self.refresh_gps_status)
        self.gps_timer.start(1000)

        self.refresh_gps_status()


    # ========================================================
    # CREATE UI
    # ========================================================

    def create_ui(self):

        central = QWidget()

        self.setCentralWidget(central)


        main_layout = QVBoxLayout(central)

        main_layout.setContentsMargins(20, 20, 20, 20)

        main_layout.setSpacing(10)


        # ====================================================
        # TITLE
        # ====================================================

        title = QLabel("ResQNet")

        title.setObjectName("title")

        main_layout.addWidget(title)


        subtitle = QLabel(
            "Raspberry Pi 5  •  RA-02 SX1278  •  ESP32 GPS"
        )

        subtitle.setObjectName("subtitle")

        main_layout.addWidget(subtitle)


        # ====================================================
        # STATUS BAR
        # ====================================================

        status_frame = QFrame()

        status_frame.setObjectName("statusFrame")


        status_layout = QHBoxLayout(status_frame)


        self.status_indicator = QLabel("●")

        self.status_indicator.setObjectName("statusRed")


        self.status_label = QLabel("Connecting to SX1278...")

        self.status_label.setObjectName("statusLabel")


        status_layout.addWidget(self.status_indicator)

        status_layout.addWidget(self.status_label)

        status_layout.addStretch()


        self.config_label = QLabel(
            "433 MHz  |  SF7  |  BW 125 kHz  |  CR 4/5  |  CRC ON"
        )

        self.config_label.setObjectName("configLabel")

        status_layout.addWidget(self.config_label)


        main_layout.addWidget(status_frame)


        # ====================================================
        # INFORMATION CARDS
        # ====================================================

        info_layout = QGridLayout()

        info_layout.setSpacing(10)


        self.node_value = self.create_info_card(
            info_layout, 0, 0, "NODE", DEFAULT_NODE_NAME
        )

        self.latitude_value = self.create_info_card(
            info_layout, 0, 1, "LAST KNOWN LAT", "—"
        )

        self.longitude_value = self.create_info_card(
            info_layout, 0, 2, "LAST KNOWN LON", "—"
        )

        self.fix_value = self.create_info_card(
            info_layout, 0, 3, "LAST GPS FIX", "—"
        )

        self.rssi_value = self.create_info_card(
            info_layout, 0, 4, "RSSI", "—"
        )

        self.snr_value = self.create_info_card(
            info_layout, 0, 5, "SNR", "—"
        )


        main_layout.addLayout(info_layout)


        # ====================================================
        # CONTENT AREA
        # ====================================================

        content_layout = QHBoxLayout()

        content_layout.setSpacing(15)


        # ====================================================
        # CHAT PANEL
        # ====================================================

        chat_frame = QFrame()

        chat_frame.setObjectName("panel")


        chat_layout = QVBoxLayout(chat_frame)


        chat_title = QLabel("MESSAGE HISTORY")

        chat_title.setObjectName("sectionTitle")

        chat_layout.addWidget(chat_title)


        self.chat_history = QTextEdit()

        self.chat_history.setReadOnly(True)

        self.chat_history.setObjectName("chatHistory")

        chat_layout.addWidget(self.chat_history)


        # ----------------------------------------------------
        # MESSAGE INPUT
        # ----------------------------------------------------

        input_layout = QHBoxLayout()


        self.message_input = QLineEdit()

        self.message_input.setPlaceholderText("Type a message...")

        self.message_input.returnPressed.connect(self.send_message)


        self.send_button = QPushButton("SEND")

        self.send_button.setObjectName("sendButton")

        self.send_button.setEnabled(False)

        self.send_button.clicked.connect(self.send_message)


        input_layout.addWidget(self.message_input)

        input_layout.addWidget(self.send_button)


        chat_layout.addLayout(input_layout)


        # ----------------------------------------------------
        # CLEAR BUTTON
        # ----------------------------------------------------

        clear_button = QPushButton("CLEAR CHAT")

        clear_button.clicked.connect(self.clear_chat)


        chat_layout.addWidget(clear_button)


        content_layout.addWidget(chat_frame, 5)


        # ====================================================
        # RIGHT PANEL
        # ====================================================

        right_frame = QFrame()

        right_frame.setObjectName("panel")


        right_layout = QVBoxLayout(right_frame)


        last_title = QLabel("LAST RECEIVED MESSAGE")

        last_title.setObjectName("sectionTitle")

        right_layout.addWidget(last_title)


        self.last_message = QLabel("No messages received.")

        self.last_message.setTextFormat(Qt.TextFormat.PlainText)

        self.last_message.setWordWrap(True)

        self.last_message.setAlignment(
            Qt.AlignmentFlag.AlignTop |
            Qt.AlignmentFlag.AlignLeft
        )

        self.last_message.setObjectName("lastMessage")

        self.last_message.setFixedHeight(68)

        right_layout.addWidget(self.last_message)


        # ----------------------------------------------------
        # OFFLINE MAP (PMTiles)
        # ----------------------------------------------------

        map_title = QLabel("OFFLINE MAP")

        map_title.setObjectName("sectionTitle")

        right_layout.addWidget(map_title)


        self.map_widget = OfflineMapWidget()

        right_layout.addWidget(self.map_widget, 1)


        # ----------------------------------------------------
        # GPS STATUS
        # ----------------------------------------------------

        gps_title = QLabel("GPS STATUS")

        gps_title.setObjectName("sectionTitle")

        right_layout.addWidget(gps_title)


        self.gps_status = QLabel("Waiting for GPS data...")

        self.gps_status.setTextFormat(Qt.TextFormat.PlainText)

        self.gps_status.setWordWrap(True)

        self.gps_status.setObjectName("gpsStatus")

        right_layout.addWidget(self.gps_status)


        content_layout.addWidget(right_frame, 6)


        main_layout.addLayout(content_layout, 1)


        # ====================================================
        # FOOTER
        # ====================================================

        footer = QLabel(
            "ResQNet  •  SX1278  •  433 MHz  •  Two-Way LoRa  •  ESP32 GPS"
        )

        footer.setAlignment(Qt.AlignmentFlag.AlignCenter)

        footer.setObjectName("footer")

        main_layout.addWidget(footer)


        # ====================================================
        # STYLE
        # ====================================================

        self.setStyleSheet("""

            QMainWindow {
                background-color: #0f172a;
            }

            QWidget {
                color: #e2e8f0;
                font-family: Arial;
            }

            QLabel#title {
                font-size: 28px;
                font-weight: bold;
                color: #f8fafc;
            }

            QLabel#subtitle {
                font-size: 14px;
                color: #94a3b8;
            }

            QFrame#statusFrame {
                background-color: #1e293b;
                border-radius: 10px;
                padding: 5px;
            }

            QLabel#statusRed {
                color: #ef4444;
                font-size: 20px;
            }

            QLabel#statusLabel {
                font-size: 15px;
                font-weight: bold;
            }

            QLabel#configLabel {
                color: #94a3b8;
                font-size: 12px;
            }

            QFrame#infoCard {
                background-color: #1e293b;
                border-radius: 10px;
            }

            QLabel#cardTitle {
                color: #94a3b8;
                font-size: 11px;
                font-weight: bold;
            }

            QLabel#cardValue {
                color: #f8fafc;
                font-size: 17px;
                font-weight: bold;
            }

            QFrame#panel {
                background-color: #1e293b;
                border-radius: 10px;
            }

            QLabel#sectionTitle {
                color: #94a3b8;
                font-size: 12px;
                font-weight: bold;
            }

            QTextEdit#chatHistory {
                background-color: #0f172a;
                border: 1px solid #334155;
                border-radius: 8px;
                padding: 8px;
                font-size: 14px;
            }

            QLineEdit {
                background-color: #0f172a;
                border: 1px solid #475569;
                border-radius: 7px;
                padding: 10px;
                color: #f8fafc;
                font-size: 14px;
            }

            QLineEdit:focus {
                border: 1px solid #38bdf8;
            }

            QPushButton {
                background-color: #334155;
                border: none;
                border-radius: 7px;
                padding: 10px 16px;
                font-weight: bold;
                color: #f8fafc;
            }

            QPushButton:hover {
                background-color: #475569;
            }

            QPushButton:pressed {
                background-color: #1e293b;
            }

            QPushButton:disabled {
                background-color: #1e293b;
                color: #64748b;
            }

            QPushButton#sendButton {
                background-color: #2563eb;
            }

            QPushButton#sendButton:hover {
                background-color: #1d4ed8;
            }

            QLabel#lastMessage {
                background-color: #0f172a;
                border-radius: 8px;
                padding: 8px;
                font-size: 14px;
            }

            QLabel#gpsStatus {
                background-color: #0f172a;
                border-radius: 8px;
                padding: 15px;
                font-size: 14px;
            }

            QLabel#footer {
                color: #64748b;
                font-size: 11px;
            }

        """)


    # ========================================================
    # CREATE INFORMATION CARD
    # ========================================================

    def create_info_card(self, grid, row, column, title, value):

        frame = QFrame()

        frame.setObjectName("infoCard")


        layout = QVBoxLayout(frame)

        layout.setContentsMargins(12, 5, 12, 5)

        layout.setSpacing(2)

        frame.setFixedHeight(62)


        title_label = QLabel(title)

        title_label.setObjectName("cardTitle")


        value_label = QLabel(value)

        value_label.setObjectName("cardValue")


        layout.addWidget(title_label)

        layout.addWidget(value_label)


        grid.addWidget(frame, row, column)


        return value_label


    # ========================================================
    # BASE MARKER ON THE OFFLINE MAP
    # ========================================================

    def show_base_on_map(self):

        # The Base needs its OWN marker. set_position() is not used
        # here because that is the node's marker and would be moved
        # away the moment the node sends a GPS fix.
        if hasattr(self.map_widget, "set_base_position"):

            self.map_widget.set_base_position(
                BASE_LATITUDE,
                BASE_LONGITUDE,
                BASE_NAME
            )

            print(
                "[MAP] Base marker placed at {:.7f}, {:.7f}".format(
                    BASE_LATITUDE,
                    BASE_LONGITUDE
                )
            )

        else:

            print(
                "[MAP] offline_map.py has no set_base_position() - "
                "Base marker not drawn"
            )

            self.chat_history.append(
                '<span style="color:#f59e0b;">'
                'SYSTEM: Base marker not drawn - offline_map.py '
                'needs a set_base_position() method.'
                '</span>'
            )


    # ========================================================
    # NODE NAME
    # ========================================================

    def update_node_name(self, name):

        name = name.strip()

        if not name or name == self.node_name:
            return


        old_name = self.node_name

        self.node_name = name

        self.node_value.setText(name)


        if old_name == DEFAULT_NODE_NAME:

            text = "SYSTEM: Node identified as <b>{}</b>".format(
                html.escape(name)
            )

        else:

            text = "SYSTEM: Node renamed <b>{}</b> → <b>{}</b>".format(
                html.escape(old_name),
                html.escape(name)
            )


        self.chat_history.append(
            '<span style="color:#a78bfa;">' + text + '</span>'
        )

        print("[NODE] Name is now: {}".format(name))

        self.refresh_gps_status()


    # ========================================================
    # GPS STATUS (runs every second + on every packet)
    # ========================================================

    def set_fix_colour(self, colour):

        self.fix_value.setStyleSheet(
            "color: {}; font-size: 17px; font-weight: bold;".format(
                colour
            )
        )


    def refresh_gps_status(self):

        base_line = "{}: {:.7f}, {:.7f}".format(
            BASE_NAME,
            BASE_LATITUDE,
            BASE_LONGITUDE
        )


        # ----------------------------------------------------
        # LINK: are we still hearing the node at all?
        # ----------------------------------------------------

        if self.last_heard is None:

            link_line = "Node not heard yet"
            link_ok = False

        else:

            since = time.time() - self.last_heard

            if since > NODE_TIMEOUT_S:

                link_line = (
                    "NODE NOT HEARD for {} - out of range or off?".format(
                        format_age(since)
                    )
                )
                link_ok = False

                if not self.link_lost:

                    self.link_lost = True

                    self.chat_history.append(
                        '<span style="color:#ef4444;">'
                        'SYSTEM: Lost contact with <b>{}</b> - '
                        'last known location kept.'
                        '</span>'.format(html.escape(self.node_name))
                    )

            else:

                link_line = "Last heard {} ago  •  RSSI {} dBm".format(
                    format_age(since),
                    self.last_rssi
                )
                link_ok = True


        # ----------------------------------------------------
        # NO LOCATION RECEIVED YET
        # ----------------------------------------------------

        if self.last_known is None:

            self.fix_value.setText("NO DATA")

            self.set_fix_colour("#94a3b8")

            if self.last_heard is None:

                first = "Waiting for {}...".format(self.node_name)

            else:

                first = "{} is online - no GPS fix yet".format(
                    self.node_name
                )

            self.gps_status.setText(
                "{}\n{}\n\n{}".format(first, link_line, base_line)
            )

            return


        # ----------------------------------------------------
        # WE HAVE A LAST KNOWN LOCATION
        # ----------------------------------------------------

        lk = self.last_known

        age = time.time() - lk["fix_time"]

        distance = distance_km(
            BASE_LATITUDE,
            BASE_LONGITUDE,
            lk["latitude"],
            lk["longitude"]
        )


        if not link_ok:

            state = "NO CONTACT - showing LAST KNOWN location"
            colour = "#ef4444"

        elif self.node_fix == "LIVE":

            state = "LIVE GPS fix"
            colour = "#22c55e"

        elif self.node_fix == "LAST":

            state = "GPS LOST on node - showing LAST KNOWN location"
            colour = "#f59e0b"

        else:

            state = "No GPS on node - showing LAST KNOWN location"
            colour = "#ef4444"


        self.fix_value.setText(format_age(age) + " ago")

        self.set_fix_colour(colour)


        self.gps_status.setText(
            "{}: {}\n"
            "{}\n\n"
            "Last known: {:.7f}, {:.7f}\n"
            "Fix age: {} ago  •  {:.2f} km from {}\n"
            "{}".format(
                self.node_name,
                state,
                link_line,
                lk["latitude"],
                lk["longitude"],
                format_age(age),
                distance,
                BASE_NAME,
                base_line
            )
        )


    # ========================================================
    # START INITIALIZATION
    # ========================================================

    def start_initialization(self):

        self.status_label.setText("Initializing SX1278...")

        QApplication.processEvents()


        try:

            self.radio.setup()

            self.radio.initialize()

        except Exception as e:

            self.radio.close()

            self.initialization_error(str(e))

            return


        self.initialization_success()


    # ========================================================
    # INITIALIZATION SUCCESS
    # ========================================================

    def initialization_success(self):

        self.status_indicator.setObjectName("statusGreen")

        self.status_indicator.setStyleSheet(
            "color: #22c55e; font-size: 20px;"
        )


        self.status_label.setText("SX1278 Connected • READY")


        self.send_button.setEnabled(True)


        self.chat_history.append(
            '<span style="color:#22c55e;">'
            'SYSTEM: LoRa radio initialized successfully.'
            '</span>'
        )


        self.chat_history.append(
            '<span style="color:#94a3b8;">'
            'Frequency: 433 MHz | SF7 | BW: 125 kHz | CR: 4/5'
            '</span>'
        )


        self.chat_history.append(
            '<span style="color:#94a3b8;">'
            '{} location: {:.7f}, {:.7f}'
            '</span>'.format(BASE_NAME, BASE_LATITUDE, BASE_LONGITUDE)
        )


        # ----------------------------------------------------
        # START RECEIVE THREAD
        # ----------------------------------------------------

        self.receive_thread = ReceiveThread(self.radio)

        self.receive_thread.message_received.connect(
            self.handle_received_message
        )

        self.receive_thread.error_occurred.connect(
            self.handle_receive_error
        )

        self.receive_thread.start()


    # ========================================================
    # INITIALIZATION ERROR
    # ========================================================

    def initialization_error(self, error):

        self.status_indicator.setObjectName("statusRed")

        self.status_label.setText("SX1278 ERROR")


        self.send_button.setEnabled(False)


        self.chat_history.append(
            '<span style="color:#ef4444;">'
            'ERROR: '
            + html.escape(error)
            + '</span>'
        )


        QMessageBox.critical(
            self,
            "LoRa Initialization Error",
            error
        )


    # ========================================================
    # HANDLE RECEIVED MESSAGE
    # ========================================================

    def handle_received_message(self, packet):

        packet_type = packet.get("type")


        # ----------------------------------------------------
        # CRC ERROR
        # ----------------------------------------------------

        if packet_type == "CRC_ERROR":

            self.chat_history.append(
                '<span style="color:#ef4444;">'
                'SYSTEM: CRC ERROR - Packet discarded.'
                '</span>'
            )

            return


        # ----------------------------------------------------
        # ANY VALID PACKET = NODE IS IN RANGE
        # ----------------------------------------------------

        self.last_heard = time.time()

        if self.link_lost:

            self.link_lost = False

            self.chat_history.append(
                '<span style="color:#22c55e;">'
                'SYSTEM: Contact with <b>{}</b> restored.'
                '</span>'.format(html.escape(packet.get("node") or self.node_name))
            )


        # ----------------------------------------------------
        # NODE NAME (carried in every new-format packet)
        # ----------------------------------------------------

        node = packet.get("node")

        if node:

            self.update_node_name(node)


        # ----------------------------------------------------
        # RSSI / SNR
        # ----------------------------------------------------

        rssi = packet.get("rssi")

        snr = packet.get("snr")


        if rssi is not None:

            self.last_rssi = rssi

            self.rssi_value.setText("{} dBm".format(rssi))


        if snr is not None:

            self.snr_value.setText("{:.2f} dB".format(snr))


        # ----------------------------------------------------
        # NAME-ONLY PACKET - nothing else to show
        # ----------------------------------------------------

        if packet_type == "NAME":

            self.refresh_gps_status()

            return


        # ----------------------------------------------------
        # GPS CHECK - every beacon and every message
        # ----------------------------------------------------

        gps_text = self.apply_gps(packet)


        # Beacons only update the GPS - they don't go in the chat
        if packet_type == "GPS_BEACON":

            return


        # ----------------------------------------------------
        # CHAT MESSAGE
        # ----------------------------------------------------

        message = packet.get("message", "")

        self.last_message.setText(
            "{}: {}".format(self.node_name, message)
        )


        self.chat_history.append(
            '<span style="color:#38bdf8;">'
            '<b>' + html.escape(self.node_name) + ':</b>'
            '</span> '
            + html.escape(message)
        )


        snr_text = (
            "{:.2f}".format(snr)
            if snr is not None
            else "—"
        )


        self.chat_history.append(
            '<span style="color:#94a3b8;">'
            + html.escape(gps_text)
            + ' | RSSI: '
            + str(rssi)
            + ' dBm | SNR: '
            + snr_text
            + ' dB'
            + '</span>'
        )


    # ========================================================
    # APPLY GPS FROM A BEACON OR A MESSAGE
    # ========================================================

    def apply_gps(self, packet):

        latitude = packet.get("latitude")

        longitude = packet.get("longitude")

        fix = packet.get("fix", "NONE")

        fix_age = packet.get("fix_age") or 0

        message = packet.get("message", "")


        previous_fix = self.node_fix

        self.node_fix = fix


        if latitude is not None and longitude is not None:

            # LAST KNOWN location - replaced by every new position,
            # never cleared, so it survives the node losing GPS
            self.last_known = {
                "latitude": latitude,
                "longitude": longitude,
                "fix_time": time.time() - fix_age,
                "fix": fix
            }


            self.latitude_value.setText("{:.7f}".format(latitude))

            self.longitude_value.setText("{:.7f}".format(longitude))


            if fix == "LAST":

                gps_text = "GPS: LAST KNOWN {:.7f}, {:.7f} ({} old)".format(
                    latitude,
                    longitude,
                    format_age(fix_age)
                )

            else:

                gps_text = "GPS: LIVE {:.7f}, {:.7f}".format(
                    latitude,
                    longitude
                )


            # pin the node on the offline map
            # (blue = live fix, amber = last known position)
            self.map_widget.set_position(
                latitude,
                longitude,
                message,
                name=self.node_name,
                fix=fix,
                fix_age=fix_age
            )

        else:

            gps_text = "GPS: No coordinates (node has no fix yet)"


        print("[GPS] {} from {}".format(gps_text, self.node_name))


        # Log only when the node's GPS state changes, not every beacon
        if fix != previous_fix:

            self.log_fix_change(fix)


        self.refresh_gps_status()

        return gps_text


    def log_fix_change(self, fix):

        name = html.escape(self.node_name)

        if fix == "LIVE":

            text = "GPS: <b>{}</b> has a live GPS fix.".format(name)
            colour = "#22c55e"

        elif fix == "LAST":

            text = (
                "GPS: <b>{}</b> LOST its GPS signal - "
                "tracking its last known location.".format(name)
            )
            colour = "#f59e0b"

        else:

            text = "GPS: <b>{}</b> has no GPS fix yet.".format(name)
            colour = "#94a3b8"

        self.chat_history.append(
            '<span style="color:{};">{}</span>'.format(colour, text)
        )


    # ========================================================
    # HANDLE RECEIVE ERROR
    # ========================================================

    def handle_receive_error(self, error):

        self.chat_history.append(
            '<span style="color:#ef4444;">'
            'RX ERROR: '
            + html.escape(error)
            + '</span>'
        )


    # ========================================================
    # SEND MESSAGE
    # ========================================================

    def send_message(self):

        message = self.message_input.text().strip()

        if not message:
            return


        if not self.radio.initialized:

            QMessageBox.warning(
                self,
                "LoRa Not Ready",
                "The SX1278 is not initialized yet."
            )

            return


        self.send_button.setEnabled(False)

        self.message_input.setEnabled(False)


        # SEND IN SEPARATE THREAD
        thread = threading.Thread(
            target=self.send_worker,
            args=(message,),
            daemon=True
        )

        thread.start()


    # ========================================================
    # SEND WORKER
    # ========================================================

    def send_worker(self, message):

        try:

            self.radio.send_message(message)

            # GUI updates must happen through signals.
            self.send_success_signal.emit(message)

        except Exception as e:

            self.send_error_signal.emit(str(e))


    # ========================================================
    # SEND SUCCESS
    # ========================================================

    def send_success(self, message):

        self.chat_history.append(
            '<span style="color:#4ade80;">'
            '<b>' + BASE_NAME + ':</b>'
            '</span> '
            + html.escape(message)
        )


        self.message_input.clear()

        self.send_button.setEnabled(True)

        self.message_input.setEnabled(True)

        self.message_input.setFocus()


    # ========================================================
    # SEND ERROR
    # ========================================================

    def send_error(self, error):

        self.chat_history.append(
            '<span style="color:#ef4444;">'
            '<b>TX ERROR:</b> '
            + html.escape(error)
            + '</span>'
        )


        self.send_button.setEnabled(True)

        self.message_input.setEnabled(True)


    # ========================================================
    # CLEAR CHAT
    # ========================================================

    def clear_chat(self):

        self.chat_history.clear()


    # ========================================================
    # CLOSE WINDOW
    # ========================================================

    def closeEvent(self, event):

        self.gps_timer.stop()


        if self.receive_thread is not None:

            self.receive_thread.stop()

            self.receive_thread = None


        self.map_widget.shutdown()


        self.radio.close()


        event.accept()


# ============================================================
# CONNECT SEND SIGNALS
# ============================================================

def connect_send_signals(window):

    window.send_success_signal.connect(window.send_success)

    window.send_error_signal.connect(window.send_error)


# ============================================================
# MAIN
# ============================================================

def main():

    app = QApplication(sys.argv)

    window = LoRaDashboard()

    connect_send_signals(window)

    window.show()

    sys.exit(app.exec())


# ============================================================
# START APPLICATION
# ============================================================

if __name__ == "__main__":

    main()
