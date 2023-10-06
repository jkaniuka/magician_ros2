from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

# ALARM CODES
# 0x00: 'reset occurred',
# 0x01: 'undefined instruction',
# 0x02: 'file system error',
# 0x03: 'communications error between MCU and FPGA',
# 0x04: 'angle sensor error',

# 0x10: 'plan: pose is abnormal',
# 0x11: 'plan: pose is out of workspace',
# 0x12: 'plan: joint limit',
# 0x13: 'plan: repetitive points',
# 0x14: 'plan: arc input parameter',
# 0x15: 'plan: jump parameter',

# 0x20: 'motion: kinematic singularity',
# 0x21: 'motion: out of workspace',
# 0x22: 'motion: inverse limit',

# 0x30: 'axis 1 overspeed',
# 0x31: 'axis 2 overspeed',
# 0x32: 'axis 3 overspeed',
# 0x33: 'axis 4 overspeed',

# 0x40: 'axis 1 positive limit',
# 0x41: 'axis 1 negative limit',
# 0x42: 'axis 2 positive limit',
# 0x43: 'axis 2 negative limit',
# 0x44: 'axis 3 positive limit',
# 0x45: 'axis 3 negative limit',
# 0x46: 'axis 4 positive limit',
# 0x47: 'axis 4 negative limit',
# 0x48: 'Parallelogram Positive Limitation Alarm',
# 0x49: 'Parallelogram Negative Limitation Alarm',

# 0x50: 'axis 1 lost steps',
# 0x51: 'axis 2 lost steps',
# 0x52: 'axis 3 lost steps',
# 0x53: 'axis 4 lost steps',

limited_position_description=[
    KeyValue(key='Reason', value="Joint position is close to the limits"),
    KeyValue(key="Solution", value="Move away from limits")]

overspeed_description=[
    KeyValue(key='Reason', value="The speed of motor is bigger than the max allowed value"),
    KeyValue(key="Solution", value="Decrease speed ratio to make joint speed smaller. Then reset alarm by using DoboControlPanel or calling ClearAlarms sevice")]

losing_step_description=[
    KeyValue(key='Reason', value="Dobot was hit during movement process or was trying to lift too heavy load"),
    KeyValue(key="Solution", value="Call ExecuteHomingProcedure service or use HOME button on DobotControlPanel")]

other_alarms_description=[
    KeyValue(key='Description', value="These alarms are software related, not hardware related, and are handled by another part of the control system. If such an alarm appeared, it is best to restart and re-power the robot and notify the maintainer of the package about it (by e-mail or via GitHub)."),
    KeyValue(key='Quick fix', value="Check alarm code in Dobot-Magician-ALARM-Description.pdf document and follow the recommendations")]


def joints_status(alarms_list):
    if 40 in alarms_list:
        diag_joint1 = warn_positive_limit('/joint1/Position')
    elif 41 in alarms_list:
        diag_joint1 = warn_negative_limit('/joint1/Position')
    else:
        diag_joint1 = ok_status('/joint1/Position')

    if 42 in alarms_list:
        diag_joint2 = warn_positive_limit('/joint2/Position')
    elif 43 in alarms_list:
        diag_joint2 = warn_negative_limit('/joint2/Position')
    else:
        diag_joint2 = ok_status('/joint2/Position')

    if 44 in alarms_list:
        diag_joint3 = warn_positive_limit('/joint3/Position')
    elif 45 in alarms_list:
        diag_joint3 = warn_negative_limit('/joint3/Position')
    else:
        diag_joint3 = ok_status('/joint3/Position')

    if 46 in alarms_list:
        diag_joint4 = warn_positive_limit('/joint4/Position')
    elif 47 in alarms_list:
        diag_joint4 = warn_negative_limit('/joint4/Position')
    else:
        diag_joint4 = ok_status('/joint4/Position')


    return [diag_joint1, diag_joint2, diag_joint3, diag_joint4]

def motors_status(alarms_list):

    # SPEED
    if 30 in alarms_list:
        diag_speed1 = warn_overspeed('/stepper1/Speed')
    else:
        diag_speed1 = ok_status('/stepper1/Speed')
    
    if 31 in alarms_list:
        diag_speed2 = warn_overspeed('/stepper2/Speed')
    else:
        diag_speed2 = ok_status('/stepper2/Speed')

    if 32 in alarms_list:
        diag_speed3 = warn_overspeed('/stepper3/Speed')
    else:
        diag_speed3 = ok_status('/stepper3/Speed')

    if 33 in alarms_list:
        diag_speed4 = warn_overspeed('/servo/Speed')
    else:
        diag_speed4 = ok_status('/servo/Speed')

    # LOST STEP DETECTION   
    if 50 in alarms_list:
        diag_lost_step1 = warn_lost_step('/stepper1/Losing-Step Detection')
    else:
        diag_lost_step1 = ok_status('/stepper1/Losing-Step Detection', "Running")
    
    if 51 in alarms_list:
        diag_lost_step2 = warn_lost_step('/stepper2/Losing-Step Detection')
    else:
        diag_lost_step2 = ok_status('/stepper2/Losing-Step Detection', "Running")

    if 52 in alarms_list:
        diag_lost_step3 = warn_lost_step('/stepper3/Losing-Step Detection')
    else:
        diag_lost_step3 = ok_status('/stepper3/Losing-Step Detection', "Running")

    if 53 in alarms_list:
        diag_lost_step4 = warn_lost_step('/servo/Losing-Step Detection')
    else:
        diag_lost_step4 = ok_status('/servo/Losing-Step Detection', "Running")


    return [diag_speed1, diag_lost_step1, diag_speed2, diag_lost_step2, diag_speed3, diag_lost_step3, diag_speed4, diag_lost_step4]

def sensors_status(alarms_list):
    if 4 not in alarms_list:
        diag_sensor = DiagnosticStatus(
            level=DiagnosticStatus.OK,
            name='/sensors/angle_sensor/Communication Status',
            message='OK')
    else:
        diag_sensor = DiagnosticStatus(
            level=DiagnosticStatus.ERROR,
            name='/sensors/angle_sensor/Communication Status',
            message='Error',
            values=[
                KeyValue(key='Reason', value="The angle sensor value can not be read correctly"),
                KeyValue(key="Solution", value="Re-power Dobot Magician, if the value is correct, that indicates reset alarm successfully")])

    return [diag_sensor]

def links_status(alarms_list):
    description=[
        KeyValue(key='Reason', value="Parallelogram is stretched to the limits"),
        KeyValue(key="Solution", value="Move away from limits")]

    if 48 in alarms_list:
        diag_link = DiagnosticStatus(
                    level=DiagnosticStatus.WARN,
                    name='/links/parallelogram/Position',
                    message='Positive limit violated',
                    values=description)
    elif 49 in alarms_list:
        diag_link = DiagnosticStatus(
                    level=DiagnosticStatus.WARN,
                    name='/links/parallelogram/Position',
                    message='Negative limit violated',
                    values=description)
    else:
         diag_link = DiagnosticStatus(
            level=DiagnosticStatus.OK,
            name='/links/parallelogram/Position',
            message='OK')

    return [diag_link]

def MCU_status(alarms_list):
    if 2 not in alarms_list:
        diag_mcu = DiagnosticStatus(
            level=DiagnosticStatus.OK,
            name='/computers/mcu/File System Status',
            message='OK')
    else:
        diag_mcu = DiagnosticStatus(
            level=DiagnosticStatus.ERROR,
            name='/computers/mcu/File System Status',
            message='Error',
            values=[
                KeyValue(key="Reason", value="Microcontroller file system error"),  
                KeyValue(key="Solution", value="Reset system using the Reset button on the back of the base. If file system is initialized successfully, the alarm is cleared automatically")])

    return [diag_mcu]

def FPGA_status(alarms_list):
    if 3 not in alarms_list:
        diag_fpga = DiagnosticStatus(
            level=DiagnosticStatus.OK,
            name='/computers/fpga/Communication with MCU Status',
            message='OK')
    else:
        diag_fpga = DiagnosticStatus(
            level=DiagnosticStatus.ERROR,
            name='/computers/fpga/Communication with MCU Status',
            message='Error',
            values=[
                KeyValue(key="Reason", value="There is a failed communication between MCU and FPGA when system is initializing"),
                KeyValue(key="Solution", value="Reset system using the Reset button on the back of the base. If the communication is successful, the alarm is cleared automatically")])

    return [diag_fpga]

def other_status(alarms_list):
    other_alarms_codes = [ 0, 1, 10, 11, 12, 13, 14, 15, 20, 21, 22]
    active_alarms = []
    for alarm in other_alarms_codes:
        if alarm in alarms_list:
            active_alarms.append(alarm)
    if not active_alarms:
        return [DiagnosticStatus(
                level = DiagnosticStatus.OK,
                name = '/other/Unhandled Exception',
                message = 'None')]
    else:
        status_list = "EXCEPTION CODES: " + str(active_alarms)[1:-1]
        return [DiagnosticStatus(
                level = DiagnosticStatus.ERROR,
                name = '/other/Unhandled Exception',
                message=status_list,
                values=other_alarms_description)]

# helpful functions (MACROS)

def ok_status(element, msg ='OK'):
    return DiagnosticStatus(
            level = DiagnosticStatus.OK,
            name = element,
            message = msg)

def warn_positive_limit(element):
    return DiagnosticStatus(
            level = DiagnosticStatus.WARN,
            name = element,
            message='Positive limit violated',
            values=limited_position_description)

def warn_negative_limit(element):
    return DiagnosticStatus(
            level = DiagnosticStatus.WARN,
            name = element,
            message='Negative limit violated',
            values=limited_position_description)

def warn_overspeed(element):
    return DiagnosticStatus(
            level = DiagnosticStatus.WARN,
            name = element,
            message='Overspeed',
            values=overspeed_description)

def warn_lost_step(element):
    return DiagnosticStatus(
            level = DiagnosticStatus.WARN,
            name = element,
            message='Missing steps',
            values=losing_step_description)