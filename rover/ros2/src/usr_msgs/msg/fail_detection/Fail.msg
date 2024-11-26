string EVENT_COLLISION = "collision"
string EVENT_FALL = "fall"
string EVENT_STUCK= "stuck"
string EVENT_BUMP= "bump"
string EVENT_ROLLOVER = "rollover"
string EVENT_CARRY = "carry"
string EVENT_TACHOMETER = "tachometer"
string EVENT_THIEF_ATTEMPT = "thief_attempt"
string EVENT_WIRELESS_BOARD_FAIL = "wireless_board_fails"


string event        # The fail event:
uint8 confidence    # The event detection confidence percentage [0-100%]
string message      # Aditional information