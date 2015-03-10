import sys, time, os
import power_control
from config import *
from pymavlink import mavutil
from math import *

class FirmwareLoadError(Exception):
    '''firmload exception class'''
    def __init__(self, msg):
        Exception.__init__(self, msg)
        self.message = msg

def show_tail(logstr):
    '''show last lines of a log'''
    ofs = logstr.tell()
    logstr.seek(0)
    lines = logstr.readlines()
    n = 20
    if len(lines) < n:
        n = len(lines)
    for i in range(n):
        print(lines[len(lines)-n+i].rstrip())
    logstr.seek(ofs)

def show_error(test, ex, logstr=None):
    '''display an error then raise an exception'''
    if logstr is not None:
        show_tail(logstr)
    raise(FirmwareLoadError("FAILED: %s (%s)" % (test, ex)))

def wait_devices(devices, timeout=10):
    '''wait for devices to appear'''
    start_time = time.time()
    missing = []
    while start_time+timeout >= time.time():
        missing = []
        all_exist = True
        for dev in devices:
            if not os.path.exists(dev):
                all_exist = False
                missing.append(dev)
        if all_exist:
            return True
        time.sleep(0.1)
    return False

def wait_no_device(devices, timeout=10):
    '''wait for devices to disappear'''
    start_time = time.time()
    while start_time+timeout >= time.time():
        found_device = False
        for dev in devices:
            if os.path.exists(dev):
                found_device = True
        if not found_device:
            return True
        time.sleep(0.1)
    return False


def failure(msg):
    '''show a failure msg and raise an exception'''
    print(msg)
    raise(FirmwareLoadError(msg))

def wait_field(refmav, msg_type, field):
    '''wait for a field value'''
    msg = None
    # get the latest available msg
    while True:
        msg2 = refmav.recv_match(type=msg_type, blocking=(msg==None), timeout=5)
        if msg2 is None:
            break
        msg = msg2
    if msg is None:
        failure("failed to reveive message %s" % msg_type)
    return getattr(msg, field)

def param_value(test, pname):
    '''get a param value given a mavproxy connection'''
    test.send('param fetch %s\n' % pname)
    test.expect('%s\s+=\s+(-?\d+\.\d+)\r\n' % pname)
    return float(test.match.group(1))

def param_set(test, pname, value):
    '''get a param value given a mavproxy connection'''
    test.send('param set %s %f\n' % (pname, value))
    test.expect('>')

def set_servo(mav, servo, value):
    '''set a servo to a value'''
    mav.mav.command_long_send(0, 0,
                              mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
                              servo, value,
                              0, 0, 0, 0, 0)

def discard_messages(mav):
    '''discard any buffered messages'''
    while True:
        msg = mav.recv_msg()
        if msg is None:
            return

def roll_estimate(RAW_IMU):
    '''estimate roll from accelerometer'''
    rx = RAW_IMU.xacc * 9.81 / 1000.0
    ry = RAW_IMU.yacc * 9.81 / 1000.0
    rz = RAW_IMU.zacc * 9.81 / 1000.0
    return degrees(-asin(ry/sqrt(rx**2+ry**2+rz**2)))

def pitch_estimate(RAW_IMU):
    '''estimate pitch from accelerometer'''
    rx = RAW_IMU.xacc * 9.81 / 1000.0
    ry = RAW_IMU.yacc * 9.81 / 1000.0
    rz = RAW_IMU.zacc * 9.81 / 1000.0
    return degrees(asin(rx/sqrt(rx**2+ry**2+rz**2)))

def attitude_estimate(RAW_IMU):
    '''return roll/pitch estimate as tuple'''
    return (roll_estimate(RAW_IMU), pitch_estimate(RAW_IMU))

def kill_processes(process_list):
    '''kill some processes by name'''
    from subprocess import call
    for p in process_list:
        call(['/usr/bin/pkill', '-9', '-f', p])

def wait_heartbeat(mav, timeout=10):
    '''wait for a heartbeat'''
    start_time = time.time()
    while time.time() < start_time+timeout:
        if mav.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5) is not None:
            return
    failure("Failed to get heartbeat")

def safety_off(mav):
    '''turn off safety switch'''
    mav.mav.set_mode_send(0, mavutil.mavlink.MAV_MODE_FLAG_DECODE_POSITION_SAFETY, 0)

def wait_mode(mav, modes, timeout=10):
    '''wait for one of a set of flight modes'''
    start_time = time.time()
    last_mode = None
    while time.time() < start_time+timeout:
        wait_heartbeat(mav, timeout=2)
        if mav.flightmode != last_mode:
            print("Flightmode %s" % mav.flightmode)
            last_mode = mav.flightmode
        if mav.flightmode in modes:
            return
    failure("Failed to get mode from %s" % modes)

class Tee(object):
    '''log to stdout and a file. Like unix tee command
    See http://stackoverflow.com/questions/616645/how-do-i-duplicate-sys-stdout-to-a-log-file-in-python
    '''
    def __init__(self, name):
        self.file = open(name, 'w')
        self.stdout = sys.stdout
        sys.stdout = self
    def __del__(self):
        self.close()
    def write(self, data):
        if self.file is not None:
            self.file.write(data)
        if self.stdout is not None:
            self.stdout.write(data)
        else:
            sys.stdout.write(data)
        self.flush()
    def flush(self):
        if self.file is not None:
            self.file.flush()
        if self.stdout is not None:
            self.stdout.flush()
        else:
            sys.stdout.flush()
    def close(self):
        if self.file is not None:
            self.file.close()
        if self.stdout is not None:
            sys.stdout = self.stdout
        self.stdout = None
        self.file = None



def mkdir_p(dir):
    '''like mkdir -p'''
    if not dir:
        return
    if dir.endswith("/"):
        mkdir_p(dir[:-1])
        return
    if os.path.isdir(dir):
        return
    mkdir_p(os.path.dirname(dir))
    os.mkdir(dir)

def gyro_vector(raw_imu):
    '''return a gyro vector in degrees/sec from a raw_imu message'''
    from pymavlink.rotmat import Vector3
    return Vector3(degrees(raw_imu.xgyro*0.001),
                   degrees(raw_imu.ygyro*0.001),
                   degrees(raw_imu.zgyro*0.001))

def emit_test_data_file(file_name, test_status, serial_number, customer, tester_name, test_process, assembly_number, assembly_revision, line, start_date, stop_date, non_required_list = {}):
	'''creates a test data file'''
	_fields = {
		'message': ['>', 1000],
		'serial': ['S', 50],
		'customer': ['C', 18],
		'board_style': ['B', 16],
		'tester_name': ['M', 16],
		'test_process': ['P', 8],
		'fixture_slot': ['s', 4],
		'fixture': ['f', 25],
		'software_document': ['d', 16],
		'software_revision': ['R', 3],
		'assembly_number': ['n', 25],
		'assembly_revision': ['r', 8],
		'assembly_version': ['v', 20],
		'firmware_revision': ['W', 25],
		'test_status': ['T', 1],
		'operator_id': ['O', 25],
		'line': ['L', 16],
		'site': ['p', 4],
		'start_date': ['[', 19],
		'stop_date': [']', 19],
		'fail_label': ['F', 25],
		'measure_label': ['M', 25],
		'measure_data': ['d', 25],
		'analysis_label': ['X', 25],
		'repair_label': ['Y', 25],
		'analysis_datetime': ['(', 8],
		'defect': ['A', 30],
		'defect_location': ['c', 25],
		'analysis_by': ['b', 25],
		'analysis_status': ['z', 1],
		'defect_pin_count': ['Q', 4],
		'defect_detail': ['a', 16],
		'part_control_number': ['G', 25],
		'machine_name': ['h', 30],
		'feeder_number': ['e', 4],
		'analysis_comment': ['m', 30],
		'rework_action': ['E', 30],
		'rework_datetime': [')', 8],
		'extra_data_keyword': ['+', 25],
		'rework_by': ['j', 25],
		'rework_comment': ['y', 255],
		'rework_status': ['w', 1],
		'rework_flag': ['t', 1],
		'analysis_confirm_date': ['u', 8],
	}

	''' pads and trims field values according to spec '''
	_to_char = lambda field_name, string: "%s%s" % (_fields[field_name][0], str(string)[:_fields[field_name][1]])

	output_fields = [_to_char('serial', serial_number),
					_to_char('customer', customer),
					_to_char('tester_name', tester_name),
					_to_char('test_process', test_process),
					_to_char('assembly_number', assembly_number),
					_to_char('assembly_version', assembly_revision),
					_to_char('line', line),
					_to_char('start_date', start_date),
					_to_char('stop_date', stop_date)]

	for key, value in non_required_list.iteritems():
		output_fields.append(_to_char(key, value))

	if test_status:
		output_fields.append('TP')
		output_fields.append(_to_char('message', '-PASS'))
	else:
		output_fields.append('TF')
		output_fields.append(_to_char('message', '-FAIL'))

	output =  "%s\r\n" % ("\r\n".join([str(x) for x in (output_fields)]))
	file = open(file_name, 'w+')
	return file.write(output)
