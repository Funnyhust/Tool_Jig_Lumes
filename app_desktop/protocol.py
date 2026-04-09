import struct

class JigProtocol:
    HEADER = 0xAA

    # Commands from firmware → PC
    CMD_TEST_START  = 0xA0
    CMD_TEST_END    = 0xA1
    CMD_SUMMARY_FRAME  = 0xA2
    
    CMD_CH_DATA_START  = 0x11
    CMD_CH_DATA_END    = 0x14

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

        # 24-byte single channel data frame (0x11 - 0x14)
        if JigProtocol.CMD_CH_DATA_START <= cmd <= JigProtocol.CMD_CH_DATA_END:
            if len(raw_bytes) < 24:
                return None
            checksum = raw_bytes[22]
            expected_cs = sum(raw_bytes[:22]) & 0xFF
            if checksum != expected_cs:
                return None

            channel_idx = cmd - JigProtocol.CMD_CH_DATA_START # 0-3
            
            # Payload layout: [2:4] V, [4:10] 3xI, [10:16] 3xP, [16:20] G, [20:22] Mask
            v_raw    = struct.unpack(">H",    raw_bytes[2:4])[0]
            currents = struct.unpack(">HHH",  raw_bytes[4:10])
            powers   = struct.unpack(">HHH",  raw_bytes[10:16])
            gain     = struct.unpack(">I",    raw_bytes[16:20])[0]
            mask     = struct.unpack(">H",    raw_bytes[20:22])[0]

            return {
                "type":       "ch_data",
                "channel":    channel_idx,
                "voltage":    v_raw / 100.0,
                "currents":   [i / 10.0  for i in currents],
                "powers":     [p / 100.0 for p in powers],
                "gain":       gain,
                "relay_mask": mask
            }

        # 108-byte summary frame
        if cmd == JigProtocol.CMD_SUMMARY_FRAME:
            if len(raw_bytes) < 108:
                return None
            checksum = raw_bytes[107]
            expected_cs = sum(raw_bytes[:107]) & 0xFF
            if checksum != expected_cs:
                return None

            # Unpack results
            zcd_ok     = list(raw_bytes[2:6])
            calib_ok   = list(raw_bytes[6:10])
            eeprom_ok  = list(raw_bytes[10:14])
            relay_ok   = list(raw_bytes[14:26])
            
            # ZCD Counts (New)
            zcd_counts = struct.unpack(">HHHH",           raw_bytes[26:34])
            
            k_u        = struct.unpack(">HHHH",           raw_bytes[34:42])
            k_i        = struct.unpack(">HHHHHHHHHHHH",   raw_bytes[42:66])
            k_p        = struct.unpack(">HHHHHHHHHHHH",   raw_bytes[66:90])
            
            cnt_u      = list(raw_bytes[90:94])
            cnt_i      = list(raw_bytes[94:106])

            return {
                "type":       "summary",
                "zcd_ok":     zcd_ok,
                "calib_ok":   calib_ok,
                "eeprom_ok":  eeprom_ok,
                "relay_ok":   relay_ok,
                "zcd_counts": list(zcd_counts),
                "k_u":        list(k_u),
                "k_i":        list(k_i),
                "k_p":        list(k_p),
                "cnt_u":      cnt_u,
                "cnt_i":      cnt_i
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
