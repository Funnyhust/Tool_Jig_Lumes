import struct

class JigProtocol:
    HEADER = 0xAA

    # Commands from firmware → PC
    CMD_TEST_START  = 0xA0
    CMD_TEST_END    = 0xA1
    CMD_DATA_FRAME  = 0x10

    # Thresholds (same as State 5 in process.cpp)
    I_ON_MIN   = 100.0  # mA: relay ON must exceed this
    P_ON_MIN   =  20.0  # W:  relay ON must exceed this
    I_OFF_MAX  = 120.0  # mA: relay OFF must be below (leakage)
    P_OFF_MAX  =  30.0  # W:  relay OFF must be below (leakage)

    @staticmethod
    def unpack_response(raw_bytes):
        if len(raw_bytes) < 5:
            return None

        header, cmd = raw_bytes[0], raw_bytes[1]
        if header != JigProtocol.HEADER:
            return None

        # 5-byte event packets: START / END
        if cmd in (JigProtocol.CMD_TEST_START, JigProtocol.CMD_TEST_END):
            checksum = raw_bytes[4]
            expected_cs = sum(raw_bytes[:4]) & 0xFF
            if checksum != expected_cs:
                return None
            return {"type": "event", "cmd": cmd}

        # 77-byte data frame
        if cmd == JigProtocol.CMD_DATA_FRAME:
            if len(raw_bytes) < 77:
                return None
            checksum = raw_bytes[76]
            expected_cs = sum(raw_bytes[:76]) & 0xFF
            if checksum != expected_cs:
                return None

            # Payload layout:
            # [2:10]  4×Voltage  uint16 ×100  → 0.01V
            # [10:34] 12×Current uint16 ×10   → 0.1mA
            # [34:58] 12×Power   uint16 ×100  → 0.01W
            # [58:74] 4×Gain     uint32
            # [74:76] relay_mask uint16 (bit0=relay0 … bit11=relay11)
            voltages   = struct.unpack(">HHHH",           raw_bytes[2:10])
            currents   = struct.unpack(">HHHHHHHHHHHH",   raw_bytes[10:34])
            powers     = struct.unpack(">HHHHHHHHHHHH",   raw_bytes[34:58])
            gains      = struct.unpack(">IIII",           raw_bytes[58:74])
            relay_mask = struct.unpack(">H",              raw_bytes[74:76])[0]

            return {
                "type":       "data",
                "voltages":   [v / 100.0  for v in voltages],   # 0.01V
                "currents":   [i / 10.0   for i in currents],   # 0.1mA
                "powers":     [p / 100.0  for p in powers],     # 0.01W
                "gains":      list(gains),
                "relay_mask": relay_mask,  # bitmask: bit N = relay N
            }

        return None

    @staticmethod
    def relay_is_on(relay_mask, relay_idx):
        """True if relay relay_idx (0-11) is currently energised."""
        return bool(relay_mask & (1 << relay_idx))

    @staticmethod
    def row_status(relay_mask, ch, r, current_ma, power_w):
        """
        Returns: 'ok' | 'fail_low' | 'fail_leak' | 'off'
        Based on relay state and measured values.
        """
        relay_idx = ch * 3 + r
        on = JigProtocol.relay_is_on(relay_mask, relay_idx)
        if on:
            if current_ma < JigProtocol.I_ON_MIN or power_w < JigProtocol.P_ON_MIN:
                return 'fail_low'
            return 'ok'
        else:
            if current_ma > JigProtocol.I_OFF_MAX or power_w > JigProtocol.P_OFF_MAX:
                return 'fail_leak'
            return 'off'
