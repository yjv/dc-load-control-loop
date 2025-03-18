import serial
import traceback

from platformio.public import DeviceMonitorFilterBase

class OutputValues(DeviceMonitorFilterBase):
    NAME = "filter_output_values"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._receiving_status = False
        self._status_bytes = []
        self._adc_conversion_factor = 1 / 15770583.04
        self._current_conversion_factor = self._adc_conversion_factor * 20.0
        self._voltage_conversion_factor = self._adc_conversion_factor * 600.0
        self._power_conversion_factor = self._adc_conversion_factor * 200.0
        self._resistance_conversion_factor = self._adc_conversion_factor * 20000.0

        self._buffer = ""

        if self.options.get("eol") == "CR":
            self._eol = "\r"
        elif self.options.get("eol") == "LF":
            self._eol = "\n"
        else:
            self._eol = "\r\n"
        print("Output values filter is loaded")

    def set_running_terminal(self, terminal):
        # force to Latin-1, issue #4732
        if terminal.input_encoding == "UTF-8":
            terminal.set_rx_encoding("Latin-1")
        if terminal.output_encoding == "UTF-8":
            terminal.set_tx_encoding("Latin-1")
        super().set_running_terminal(terminal)

    def rx(self, text):
        try:
            result = ""

            for text_byte in serial.iterbytes(text):

                if text_byte == '\xFF' and not self._receiving_status:
                    self._receiving_status = True
                    self._status_bytes = []
                    continue

                if self._receiving_status:
                    self._status_bytes.append(ord(text_byte))
                    lens = len(self._status_bytes)
                    if len(self._status_bytes) == 28:
                        voltage_code = int.from_bytes(self._status_bytes[0:4])
                        current_code = int.from_bytes(self._status_bytes[4:8])
                        setpoint_code = int.from_bytes(self._status_bytes[8:12])
                        mode_code = int.from_bytes(self._status_bytes[12:16])
                        reading_count = int.from_bytes(self._status_bytes[16:20])
                        invalid_count = int.from_bytes(self._status_bytes[20:24])
                        update_dac_count = int.from_bytes(self._status_bytes[24:28])

                        voltage = float(voltage_code) * self._voltage_conversion_factor
                        current = float(current_code) * self._current_conversion_factor
                        power = voltage * current
                        resistance = voltage / max(current, 0.000001)
                        
                        if mode_code == 0:
                            mode = "Current"
                            setpoint_suffix = "A"
                            conversion_factor = self._current_conversion_factor
                        elif mode_code == 1:
                            mode = "Voltage"
                            setpoint_suffix = "V"
                            conversion_factor = self._voltage_conversion_factor
                        elif mode_code == 2:
                            mode = "Power"
                            setpoint_suffix = "W"
                            conversion_factor = self._power_conversion_factor
                        elif mode_code == 3:
                            mode = "Resistance"
                            setpoint_suffix = "Ω"
                            conversion_factor = self._resistance_conversion_factor
                        else:
                            mode = "Unknown"

                        setpoint = float(setpoint_code) * conversion_factor

                        result += f"Current: {current_code:8d} {self._nicify(current)}A, Voltage: {voltage_code:8d} {self._nicify(voltage)}V, Power: {self._nicify(power)}W, Resistance: {self._nicify(resistance)}Ω, Setpoint: {setpoint_code:8d} {self._nicify(setpoint)}{setpoint_suffix}\n Reading Count: {reading_count}, Invalid Count: {invalid_count}, Update DAC Count: {update_dac_count}\n"
                        self._receiving_status = False
                else:
                    result += text_byte

            return result
        except Exception as e:
            print(traceback.format_exc())
            raise e

    def _nicify(self, value):
        if value > 1000000:
            return f"{value/1000000:9.12f}M"
        if value > 1000:
            return f"{value/1000:9.9f}k"
        if value < 1 and value >= 0.001:
            return f"{value * 1000:9.3f}m"
        if value < 0.001 and value >= 0.000001:
            return f"{value * 1000000:9.0f}u"
        return f"{value:9.6f}"

    def tx(self, text):
        self._buffer += text
        if self._buffer.endswith(self._eol):
            text = self._buffer[:-len(self._eol)]
            self._buffer = ""

            print(f"Received command: {text}")

            data = [0];

            if text[-1:].lower() == "a":
                data.append(0)
                conversion_factor = self._current_conversion_factor
            elif text[-1:].lower() == "v":
                data.append(1)
                conversion_factor = self._voltage_conversion_factor
            elif text[-1:].lower() == "w":
                data.append(2)
                conversion_factor = self._power_conversion_factor
            elif text[-1:].lower() == "o" or text[-1:].lower() == "Ω":
                data.append(3)
                conversion_factor = self._resistance_conversion_factor
            else:
                print(f"Invalid command {ord(text[-1:])}")
                return ""
            
            number_end = -2

            if text[-2:-1] == "m":
                multiplier = 0.001
            elif text[-2:-1] == "u":
                multiplier = 0.000001
            elif text[-2:-1] == "k":
                multiplier = 1000
            elif text[-2:-1] == "M":
                multiplier = 1000000
            else:
                number_end = -1
                multiplier = 1
            
            value = int(float(text[:number_end]) * multiplier / conversion_factor)

            print(f"Value: {value}")
            value_bytes = value.to_bytes(4)
            data += value_bytes

            print(f"Sending command: {data}")

            print(ord("".join([chr(data_byte) for data_byte in data])[-1]))

            return "".join([chr(data_byte) for data_byte in data])
        return ""

