#!/bin/bash

# Defaults
BUS=""
ADDR="0x10"

# Regs
REG_COMMAND="0x00"
REG_MOVE_DIST="0x01"
REG_STATUS="0x02"
REG_MODE="0x03"
REG_MOTOR="0x04"
REG_PARAM_SPEED="0x05"
REG_PARAM_TIMEOUT="0x06"
REG_PARAM_EMPTYING_TIMEOUT="0x07"
REG_PARAM_HOLD_TIMEOUT="0x08"
REG_PARAM_HOLD_TIMEOUT_ENABLED="0x09"
REG_PARAM_MULTI_PRESS_COUNT="0x0A"

# Commands
CMD_OFF="0x00"
CMD_REGULAR="0x01"
CMD_HOLD="0x02"
CMD_PUSH="0x03"
CMD_RETRACT="0x04"

usage() {
    echo "Usage: $0 -b <bus> [-a <addr>] <command> [args]"
    echo "Commands:"
    echo "  push              - Push filament (Continuous)"
    echo "  retract           - Retract filament (Continuous)"
    echo "  hold              - Hold filament"
    echo "  regular           - Regular mode"
    echo "  off               - Motor off"
    echo "  move <dist>       - Move distance (mm)"
    echo "  status            - Read status"
    echo "  speed [val]       - Get or set speed (mm/s)"
    echo "  timeout [val]     - Get or set timeout (ms)"
    echo "  emptying_timeout [val] - Get or set emptying timeout (ms)"
    echo "  hold_timeout [val] - Get or set hold timeout (ms)"
    echo "  timeout_en [val]  - Get or set hold timeout enabled"
    echo "  multi_press [val] - Get or set multi press count"
    echo "  params            - Read all parameters"
    exit 1
}

# Parse Args
while getopts "b:a:" opt; do
    case $opt in
        b) BUS="$OPTARG";;
        a) ADDR="$OPTARG";;
        *) usage;;
    esac
done
shift $((OPTIND-1))

if [ -z "$BUS" ]; then
    echo "Error: Bus number required (-b)"
    usage
fi

CMD="$1"
shift

# Helper to pack float to bytes
float_to_bytes() {
    python3 -c "import struct; print(' '.join(map(str, struct.pack('<f', float($1)))))"
}

# Helper to pack uint32 to bytes
uint32_to_bytes() {
    python3 -c "import struct; print(' '.join(map(str, struct.pack('<I', int($1)))))"
}

# Helper to unpack bytes to float
bytes_to_float() {
    python3 -c "import struct, sys; data=bytes(map(int, sys.argv[1:])); print(f'{struct.unpack(\"<f\", data)[0]:.2f}')" "$@"
}
uint32_to_bytes() {
    python3 -c "import struct; print(' '.join(map(str, struct.pack('<I', int($1)))))"
}
bytes_to_uint32() {
    python3 -c "import struct, sys; data=bytes(map(int, sys.argv[1:])); print(struct.unpack(\"<I\", data)[0])" "$@"
}

case "$CMD" in
    push)
        i2cset -y "$BUS" "$ADDR" "$REG_COMMAND" "$CMD_PUSH"
        ;;
    retract)
        i2cset -y "$BUS" "$ADDR" "$REG_COMMAND" "$CMD_RETRACT"
        ;;
    hold)
        i2cset -y "$BUS" "$ADDR" "$REG_COMMAND" "$CMD_HOLD"
        ;;
    regular)
        i2cset -y "$BUS" "$ADDR" "$REG_COMMAND" "$CMD_REGULAR"
        ;;
    off)
        i2cset -y "$BUS" "$ADDR" "$REG_COMMAND" "$CMD_OFF"
        ;;
    move)
        if [ -z "$1" ]; then echo "Missing distance"; exit 1; fi
        BYTES=$(float_to_bytes "$1")
        # i2cset block write: register then bytes. i mode for block
        i2cset -y -i "$BUS" "$ADDR" "$REG_MOVE_DIST" $BYTES
        ;;
    status)
        # Read Status
        VAL=$(i2cget -y "$BUS" "$ADDR" "$REG_STATUS")
        FILAMENT_PRESENT=$(($VAL & 0x01))
        TIMED_OUT=$(( ($VAL >> 1) & 0x01 ))
        HOLD_TIMEOUT_EN=$(( ($VAL >> 2) & 0x01 ))

        printf "Status: (%s)\n" "$VAL"
        printf "  - Filament Present: %s\n" "$FILAMENT_PRESENT"
        printf "  - Timed Out: %s\n" "$TIMED_OUT"
        
        # Read Mode
        VAL=$(i2cget -y "$BUS" "$ADDR" "$REG_MODE")
        MODE_NAME="Unknown"
        case "$VAL" in
            0x00) MODE_NAME="Regular";;
            0x01) MODE_NAME="Continuous";;
            0x02) MODE_NAME="MoveCmd";;
            0x03) MODE_NAME="Hold";;
            0x04) MODE_NAME="Manual";;
        esac
        printf "Mode:   %s (%s)\n" "$VAL" "$MODE_NAME"
        
        # Read Motor
        VAL=$(i2cget -y "$BUS" "$ADDR" "$REG_MOTOR")
        MOT_NAME="Unknown"
        case "$VAL" in
            0x00) MOT_NAME="Push";;
            0x01) MOT_NAME="Retract";;
            0x02) MOT_NAME="Hold";;
            0x03) MOT_NAME="Off";;
        esac
        printf "Motor:  %s (%s)\n" "$VAL" "$MOT_NAME"
        ;;
    params)
        printf "--- Settings ---\n"
        # Speed
        VAL=$(i2cget -y "$BUS" "$ADDR" "$REG_PARAM_SPEED" i 4 | cut -d: -f2)
        printf "Speed:            %s mm/s\n" "$(bytes_to_float $VAL)"
        
        # Timeout
        VAL=$(i2cget -y "$BUS" "$ADDR" "$REG_PARAM_TIMEOUT" i 4 | cut -d: -f2)
        printf "Timeout:          %s ms\n" "$(bytes_to_uint32 $VAL)"
        
        # Emptying Timeout
        VAL=$(i2cget -y "$BUS" "$ADDR" "$REG_PARAM_EMPTYING_TIMEOUT" i 4 | cut -d: -f2)
        printf "Emptying Timeout: %s ms\n" "$(bytes_to_uint32 $VAL)"
        
        # Hold Timeout
        VAL=$(i2cget -y "$BUS" "$ADDR" "$REG_PARAM_HOLD_TIMEOUT" i 4 | cut -d: -f2)
        printf "Hold Timeout:     %s ms\n" "$(bytes_to_uint32 $VAL)"
        
        # Hold Timeout En
        VAL=$(i2cget -y "$BUS" "$ADDR" "$REG_PARAM_HOLD_TIMEOUT_ENABLED")
        printf "Hold Timeout En:  %s\n" "$VAL"
        
        # Multi-Press
        VAL=$(i2cget -y "$BUS" "$ADDR" "$REG_PARAM_MULTI_PRESS_COUNT")
        printf "Multi-Press:      %s\n" "$VAL"
        ;;
    speed)
        if [ -z "$1" ]; then
            VAL=$(i2cget -y "$BUS" "$ADDR" "$REG_PARAM_SPEED" i 4 | cut -d: -f2)
            bytes_to_float $VAL
        else
            BYTES=$(float_to_bytes "$1")
            i2cset -y "$BUS" "$ADDR" "$REG_PARAM_SPEED" $BYTES
        fi
        ;;
    timeout)
        if [ -z "$1" ]; then
            VAL=$(i2cget -y "$BUS" "$ADDR" "$REG_PARAM_TIMEOUT" i 4 | cut -d: -f2)
            bytes_to_uint32 $VAL
        else
            BYTES=$(uint32_to_bytes "$1")
            i2cset -y "$BUS" "$ADDR" "$REG_PARAM_TIMEOUT" $BYTES
        fi
        ;;
    emptying_timeout)
        if [ -z "$1" ]; then
            VAL=$(i2cget -y "$BUS" "$ADDR" "$REG_PARAM_EMPTYING_TIMEOUT" i 4 | cut -d: -f2)
            bytes_to_uint32 $VAL
        else
            BYTES=$(uint32_to_bytes "$1")
            i2cset -y "$BUS" "$ADDR" "$REG_PARAM_EMPTYING_TIMEOUT" $BYTES
        fi
        ;;
    hold_timeout)
        if [ -z "$1" ]; then
            VAL=$(i2cget -y "$BUS" "$ADDR" "$REG_PARAM_HOLD_TIMEOUT" i 4 | cut -d: -f2)
            bytes_to_uint32 $VAL
        else
            BYTES=$(uint32_to_bytes "$1")
            i2cset -y "$BUS" "$ADDR" "$REG_PARAM_HOLD_TIMEOUT" $BYTES
        fi
        ;;
    timeout_en)
        if [ -z "$1" ]; then
            i2cget -y "$BUS" "$ADDR" "$REG_PARAM_HOLD_TIMEOUT_ENABLED"
        else
            i2cset -y "$BUS" "$ADDR" "$REG_PARAM_HOLD_TIMEOUT_ENABLED" "$1"
        fi
        ;;
    multi_press)
        if [ -z "$1" ]; then
            i2cget -y "$BUS" "$ADDR" "$REG_PARAM_MULTI_PRESS_COUNT"
        else
            i2cset -y "$BUS" "$ADDR" "$REG_PARAM_MULTI_PRESS_COUNT" "$1"
        fi
        ;;
    *)
        usage
        ;;
esac
