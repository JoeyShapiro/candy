logf = open('candy-bar/logs/device-monitor-261010-164259.log')
dbcf = open('honda.dbc')

# https://github.com/commaai/opendbc/tree/master/opendbc/dbc
# https://www.csselectronics.com/pages/can-dbc-file-database-intro
# not exactly what i wanted. but i do get what i need. still no tx

ids = {}
for line in dbcf.readlines():
    if not line.startswith('BO_'):
        continue

    # split the line into fields
    fields = line.split()

    # store the message ID and name in the dictionary
    ids[int(fields[1])] = fields[2].strip(':')


for line in logf.readlines():
    # split the line into fields
    # (don't key off leading spaces: the timestamp is right-aligned, so
    # there are 4 before 10s, 3 before 100s, 2 after)
    fields = line.split()

    # skip lines where fields[1] is not RX/TX
    if len(fields) < 4 or fields[1] not in ['RX', 'TX']:
        continue

    # convert hex id to int
    try:
        pid = int(fields[2], 16)
        if pid in ids:
            print(fields[0], fields[1], ids[pid])

            if pid == 344: # engine data
                # data bytes come after the DLC field
                data = [int(b, 16) for b in fields[4:]]
                if len(data) < 8:
                    print("    short frame:", ' '.join(fields[4:]))
                    continue

                # big-endian (@0) signals, byte offsets from the DBC start bits
                xmission_speed = (data[0] << 8 | data[1]) * 0.01   # 7|16
                engine_rpm = data[2] << 8 | data[3]                 # 23|16
                xmission_speed2 = (data[4] << 8 | data[5]) * 0.01  # 39|16
                odometer = data[6] * 10                             # 55|8
                counter = (data[7] >> 4) & 0x3                      # 61|2
                checksum = data[7] & 0xF                            # 59|4

                print(f"    speed:    {xmission_speed:6.2f} kph ({xmission_speed * 0.621371:6.2f} mph)")
                print(f"    speed 2:  {xmission_speed2:6.2f} kph")
                print(f"    rpm:      {engine_rpm} rpm")
                print(f"    odometer: {odometer} m")
                print(f"    counter:  {counter}  checksum: 0x{checksum:X}")

            if pid == 380: # powertrain data
                data = [int(b, 16) for b in fields[4:]]
                if len(data) < 8:
                    print("    short frame:", ' '.join(fields[4:]))
                    continue

                pedal_gas = data[0]                                 # 7|8
                engine_rpm = data[2] << 8 | data[3]                 # 23|16
                gas_pressed = data[4] >> 7 & 0x1                    # 39|1
                acc_status = data[4] >> 6 & 0x1                     # 38|1
                boh_17c = data[4] >> 1 & 0x1F                       # 37|5
                brake_switch = data[4] & 0x1                        # 32|1
                boh2_17c = data[5] << 2 | data[6] >> 6              # 47|10
                brake_pressed = data[6] >> 5 & 0x1                  # 53|1
                boh3_17c = data[6] & 0x1F                           # 52|5
                counter = (data[7] >> 4) & 0x3                      # 61|2
                checksum = data[7] & 0xF                            # 59|4

                yes_no = lambda b: "yes" if b else "no"
                print(f"    rpm:           {engine_rpm} rpm")
                print(f"    gas pedal:     {pedal_gas} ({pedal_gas * 100 / 255:.0f}%)")
                print(f"    gas pressed:   {yes_no(gas_pressed)}")
                print(f"    brake pressed: {yes_no(brake_pressed)}  brake switch: {yes_no(brake_switch)}")
                print(f"    acc on:        {yes_no(acc_status)}")
                print(f"    unknown:       boh={boh_17c} boh2={boh2_17c} boh3={boh3_17c}")
                print(f"    counter:       {counter}  checksum: 0x{checksum:X}")
    except ValueError:
        pass
