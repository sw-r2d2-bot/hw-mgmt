#!/usr/bin/python
"""
########################################################################
# Copyright (c) 2019 Mellanox Technologies. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the names of the copyright holders nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# Alternatively, this software may be distributed under the terms of the
# GNU General Public License ("GPL") version 2 as published by the Free
# Software Foundation.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

# Thermal configuration per system type. The next types are supported:
#  t1: MSN27*|MSN24*        Panther, Spider
#  t2: MSN21*            Bulldog
#  t3: MSN274*            Panther SF
#  t4: MSN201*            Boxer
#  t5: MSN27*|MSB*|MSX*        Neptune, Tarantula, Scorpion, Scorpion2
#  t6: QMB7*|SN37*|SN34*    Jaguar, Anaconda

# The thermal algorithm considers the next rules for FAN speed setting:
# The minimal PWM setting is dynamic and depends on FAN direction and cable
# type. For system with copper cables only or/and with trusted optic cable
# minimum PWM setting could be decreased according to the system definition.
# Power supply units PWM control policy:
# If system`s power supplies are equipped with the controlled cooling device,
# its cooling setting should follow the next rules
# - Power supplies cooling devices should be set to the default value
#  (usually 60% of PWM speed), defined per each system type;
# - In case system`s main cooling device is set above this default value, power
#   supply`s cooling device setting should follow main cooling device (for
#   example if main cooling device is set to 80%, power supplies cooling devices
#   should be set to 80%);
# - In case system`s main cooling device is set down (to x%), power supplys'
#   cooling devices should be set down, in case x% >= power supplies default
#   tachometers is broken (in such case thermal monitoring in kernel is set to
#   disabled state until the problem is not recovered). Such events will be
#   reported to systemd journaling system.
# Thermal active monitoring is performed based on the values of the next
# sensors: CPU, ASIC ambient and QSFP modules temperatures.
# The decision for PWM setting is taken based on the worst measure of them.
# All the sensors and statuses are exposed through the sysfs interface for the
# user space application access.
"""

#######################################################################
# Global imports
#######################################################################
import os
import sys
import time
import traceback
import argparse
import subprocess
import signal
import logging
import syslog
import atexit
from threading import Timer


#############################
# Global const
#############################
#pylint: disable=c0301,W0105

# Local constants
class HWConst(object):
    '''
    @summary: hw-management constants
    '''
    PWM_NOACT = 0
    PWM_MAX = 1
    PWM_MAX_RPM = 255
    PWM_DEF_RPM = 153
    FAN_MAX_STATE = 10
    THERMAL_POLL_TIME = 30
    REPORT_POLL_TIME = 90
    FAN_DYN_MIN_FAIL = 20
    HW_MGMT_FOLDER = '/var/run/hw-management'
    HW_MGMT_PID = '/var/run/thermal-control.pid'
    CONFIG_FILE = '/var/run/hw-management/config/thermal_config'
    WAIT_START = 120
    DEVNULL = '/dev/null'
    TEMP_TABLE_SCALE = 1000

# PSU fan speed vector
PSU_FAN_SPEED = ['0x3c', '0x3c', '0x3c', '0x3c', '0x3c',
                 '0x3c', '0x3c', '0x46', '0x50', '0x5a', '0x64']

#############################
# Global variables
#############################

"""
Thermal tables for the minimum FAN setting per system type. It contains
entries with ambient temperature threshold values and relevant minimum
speed setting. All Mellanox system are equipped with two ambient sensors:
port side ambient sensor and FAN side ambient sensor. FAN direction can
be read from FAN EEPROM data, in case FAN is equipped with EEPROM device,
it can be read from CPLD FAN direction register in other case. Or for the
common case it can be calculated according to the next rule:
if port side ambient sensor value is greater than FAN side ambient sensor
value - the direction is power to cable (forward); if it less - the direction
is cable to power (reversed), if these value are equal: the direction is
unknown. For each system the following six tables are defined:
p2c_dir_trust_tx    all cables with trusted or with no sensors, FAN
        direction is power to cable (forward)
p2c_dir_untrust_tx    some cable sensor is untrusted, FAN direction is
        power to cable (forward)
c2p_dir_trust_tx    all cables with trusted or with no sensors, FAN
        direction is cable to power (reversed)
c2p_dir_untrust_tx    some cable sensor is untrusted, FAN direction is
        cable to power (reversed)
unk_dir_trust_tx    all cables with trusted or with no sensors, FAN
        direction is unknown
unk_dir_untrust_tx    some cable sensor is untrusted, FAN direction is
        unknown
The below tables are defined per system thermal class and defines the
relationship between the ambient temperature and minimal FAN speed. Th
minimal FAN speed is coded as following: 12 for 20%, 13 for 30%, ..., 19 for
90%, 20 for 100%.
"""

"""
Class t1 for MSN27*|MSN24* (Panther, Spider)
Direction    P2C        C2P        Unknown
-------------------------------------------------------------
Amb [C]    copper/    AOC W/O copper/    AOC W/O    copper/    AOC W/O
    sensors    sensor    sensor    sensor    sensor    sensor
-------------------------------------------------------------
 <0           30    30    30    30    30    30
 0-5          30    30    30    30    30    30
 5-10         30    30    30    30    30    30
 10-15        30    30    30    30    30    30
 15-20        30    30    30    30    30    30
 20-25        30    30    40    40    40    40
 25-30        30    40    50    50    50    50
 30-35        30    50    60    60    60    60
 35-40        30    60    60    60    60    60
 40-45        50    60    60    60    60    60
"""

TABLE_CLASS1 = {
    "p2c_trust":   {"-127:120":13},
    "p2c_untrust": {"-127:25":13, "26:31":14 , "31:35":15, "36:120":16},
    "c2p_trust":   {"-127:20":13, "21:25":14 , "26:31":15, "31:120":16},
    "c2p_untrust": {"-127:20":13, "21:25":14 , "26:31":15, "31:120":16},
    "unk_trust":   {"-127:20":13, "21:25":14 , "26:31":15, "31:120":16},
    "unk_untrust": {"-127:20":13, "21:25":14 , "26:31":15, "31:120":16}
}

"""
MSN21* (Bulldog)
Direction    P2C        C2P        Unknown
--------------------------------------------------------------
 Amb [C]    copper/    AOC W/O copper/    AOC W/O    copper/    AOC W/O
        sensors    sensor    sensor    sensor    sensor    sensor
--------------------------------------------------------------
  <0          20    20    20    20    20    20
  0-5         20    20    20    20    20    20
  5-10        20    20    20    20    20    20
 10-15        20    20    20    20    20    20
 15-20        20    30    20    20    20    30
 20-25        20    30    20    20    20    30
 25-30        20    40    20    20    20    40
 30-35        20    50    20    20    20    50
 35-40        20    60    20    20    20    60
 40-45        20    60    30    30    30    60
"""

TABLE_CLASS2 = {
    "p2c_trust":  {"-127:120":12},
    "p2c_untrust": {"-127:15":12, "16:25":13, "26:31":14, "31:35":15, "36:120":16},
    "c2p_trust":   {"-127:14":12, "41:120":13},
    "c2p_untrust": {"-127:14":12, "41:120":13},
    "unk_trust":   {"-127:14":12, "41:120":13},
    "unk_untrust": {"-127:15":12, "16:25":13, "26:31":14, "31:35":15, "36:120":16}
}


"""
Class t3 for MSN274* (Panther SF)
Direction    P2C        C2P        Unknown
-------------------------------------------------------------
Amb [C]    copper/    AOC W/O copper/    AOC W/O    copper/    AOC W/O
    sensors    sensor    sensor    sensor    sensor    sensor
-------------------------------------------------------------
 <0          30    30    30    30    30    30
 0-5         30    30    30    30    30    30
 5-10        30    30    30    30    30    30
10-15        30    30    30    30    30    30
15-20        30    30    30    40    30    40
20-25        30    30    30    40    30    40
25-30        30    30    30    40    30    40
30-35        30    30    30    50    30    50
35-40        30    40    30    70    30    70
40-45        30    50    30    70    30    70
"""

TABLE_CLASS3 = {
    "p2c_trust":   {"-127:120":13},
    "p2c_untrust": {"-127:35":13, "36:14":14 , "41:120":15},
    "c2p_trust":   {"-127:120":13},
    "c2p_untrust": {"-127:15":13, "16:31":14 , "31:35":15, "36:120":17},
    "unk_trust":   {"-127:120":13},
    "unk_untrust": {"-127:15":13, "16:31":14 , "31:35":15, "36:120":17},
    "trust":   30,
    "untrust": 70
}

'''
 Class t4 for MSN201* (Boxer)
 Direction    P2C        C2P        Unknown
--------------------------------------------------------------
 Amb [C]    copper/    AOC W/O copper/    AOC W/O    copper/    AOC W/O
        sensors    sensor    sensor    sensor    sensor    sensor
--------------------------------------------------------------
  <0          20    20    20    20    20    20
  0-5         20    20    20    20    20    20
  5-10        20    20    20    20    20    20
 10-15        20    20    20    20    20    20
 15-20        20    30    20    20    20    30
 20-25        20    40    20    30    20    40
 25-30        20    40    20    40    20    40
 30-35        20    50    20    50    20    50
 35-40        20    60    20    60    20    60
 40-45        20    60    20    60    20    60
'''

TABLE_CLASS4 = {
    "p2c_trust":   {"-127:120":20},
    "p2c_untrust": {"-127:10":20, "11:15":13, "16:20":14, "21:31":15, "31:120":16},
    "c2p_trust":   {"-127:120":20},
    "c2p_untrust": {"-127:20":20, "21:25":13 , "26:31":14, "31:35":15, "36:120":16},
    "unk_trust":   {"-127:120":20},
    "unk_untrust": {"-127:10":20, "11:15":13 , "16:20":14, "11:31":15, "31:120":16}
}

'''
 Class t5 for MSN370* (Anaconda)
 Direction    P2C        C2P        Unknown
--------------------------------------------------------------
 Amb [C]    copper/    AOC W/O copper/    AOC W/O    copper/    AOC W/O
        sensors    sensor    sensor    sensor    sensor    sensor
--------------------------------------------------------------
  <0          20    20    20    20    20    20
  0-5         20    20    20    20    20    20
  5-10        20    20    20    20    20    20
 10-15        20    20    20    20    20    20
 15-20        20    30    20    20    20    30
 20-25        20    30    20    20    20    30
 25-30        30    30    30    30    30    30
 30-35        30    40    30    30    30    40
 35-40        30    50    30    30    30    50
 40-45        40    60    40    40    40    60
'''

TABLE_CLASS5 = {
    "p2c_trust":   {"-127:20":20, "21:25":13 , "26:120":14},
    "p2c_untrust": {"-127:10":20, "11:25":13 , "26:31":14, "31:35":15, "36:120":16},
    "c2p_trust":   {"-127:20":20, "21:31":13 , "31:120":14},
    "unk_trust":   {"-127:20":20, "26:120":14},
    "unk_untrust": {"-127:10":20, "11:25":13 , "26:31":14, "31:35":15, "36:120":16},
    "trust":   20,
    "untrust": 60
}

THERMAL_TABLE_LIST = {
                   "TC1": {"table" : TABLE_CLASS1, "trust":16, "untrust":16},
                   "TC2": {"table" : TABLE_CLASS2, "trust":13, "untrust":16},
                   "TC3": {"table" : TABLE_CLASS3, "trust":13, "untrust":17},
                   "TC4": {"table" : TABLE_CLASS4, "trust":12, "untrust":16},
                   "TC5": {"table" : TABLE_CLASS5, "trust":12, "untrust":16}
                 }



class Logger(object):
    '''
    Logger class provide functionality to log messages.
    It can log to several places in parallel
    '''

    def __init__(self, use_syslog='0', log_file=None):
        '''
        @summary:
            The following calss provide functionality to log messages.
            log provided by /lib/lsb/init-functions always turned on
        @param use_syslog: log also to syslog. Applicable arg
            value 1-enable/0-disable
        @param log_file: log to user specified file. Set '' if no log needed
        '''
        self.logger_fh = None
        self.logger = None
        self.log_file = None
        self.use_syslog = None
        self.set_param(use_syslog, log_file)

    def __del__(self):
        '''
        @summary:
            Close log handlers if needed
         '''
        if self.logger:
            if self.logger_fh:
                self.logger.removeHandler(self.logger_fh)
                self.logger_fh.close()

    def set_param(self, use_syslog=None, log_file=None):
        '''
        @summary:
            Set logger parameters. Can be calld any time
            log provided by /lib/lsb/init-functions always turned on
        @param use_syslog: log also to syslog. Applicable arg
            value 1-enable/0-disable
        @param log_file: log to user specified file. Set None if no log needed
        '''
        res = 0
        if log_file:
            if self.logger:
                if self.logger_fh:
                    self.logger.removeHandler(self.logger_fh)
                    self.logger_fh.close()

            self.logger = logging.getLogger("main")
            self.logger.setLevel(logging.INFO)
            # create the logging file handler
            self.logger_fh = logging.FileHandler(log_file)
            formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
            self.logger_fh.setFormatter(formatter)

            # add handler to logger object
            self.logger.addHandler(self.logger_fh)
            self.log_file = log_file

        if use_syslog is not None:
            self.use_syslog = use_syslog
        return res

    def action(self, msg=''):
        '''
        @summary:
            Log "action" message.
        @param msg: message to save to log
        '''
        if self.logger:
            self.logger.info(msg)
        if self.use_syslog == '1':
            syslog.syslog(syslog.LOG_INFO, msg)
        subprocess.call(". /lib/lsb/init-functions && log_action_msg '{0}'".format(msg), shell=True)

    def success(self, msg=''):
        '''
        @summary:
            Log "success" message.
        @param msg: message to save to log
        '''
        if self.logger:
            self.logger.info(msg)
        if self.use_syslog == '1':
            syslog.syslog(syslog.LOG_INFO, msg)

        subprocess.call(". /lib/lsb/init-functions && log_success_msg '{0}'".format(msg), shell=True)

    def failure(self, msg=''):
        '''
        @summary:
            Log "failure" message.
        @param msg: message to save to log
        '''
        if self.logger:
            self.logger.error(msg)
        if self.use_syslog == '1':
            syslog.syslog(syslog.LOG_ERR, msg)
        subprocess.call(". /lib/lsb/init-functions && log_failure_msg '{0}'".format(msg), shell=True)

    def warn(self, msg=''):
        '''
        @summary:
            Log "warn" message.
        @param msg: message to save to log
        '''
        if self.logger:
            self.logger.warning(msg)
        if self.use_syslog == '1':
            syslog.syslog(syslog.LOG_WARNING, msg)
        subprocess.call(". /lib/lsb/init-functions && log_warning_msg '{0}'".format(msg), shell=True)

class RepeatedTimer(object):
    '''
     @summary:
         Provide repeat timer service. Can start provided function with selected  interval
    '''
    def __init__(self, interval, function):
        '''
        @summary:
            Create timer object which run function in separate thread
            Automatically start timer after init
        @param interval: loInterval in seconds to run function
        @param function: function name to run
        '''
        self._timer     = None
        self.interval   = interval
        self.function   = function

        self.is_running = False
        self.start()

    def _run(self):
        '''
        @summary:
            wrapper to run function
        '''
        self.is_running = False
        self.start()
        self.function()

    def start(self):
        '''
        @summary:
            Start selected timer (if it not running)
        '''
        if not self.is_running:
            self._timer = Timer(self.interval, self._run)
            self._timer.start()
            self.is_running = True

    def stop(self):
        '''
        @summary:
            Stop selected timer (if it started before
        '''
        self._timer.cancel()
        self.is_running = False

class ThermalManagement(object):
    '''
        @summary:
            Main class of thermal algorithm.
            Provide system monitoring and thermal control
    '''
    def __init__(self, thermal_class, max_tachos, max_psus,
                root_folder = HWConst.HW_MGMT_FOLDER, use_syslog = 0, log_file = ''):
        '''
        @summary:
        @param thermal_class: Thermal device class.
        @param max_tachos: Number of FAN tacho sensors
        @param max_psus: Number of PSU in system
        @param use_syslog: Log thermal algo messages also into syslog
        @param root_folder: path to hw-management root folder.
        @param log_file: Log thermal algo messages also into user specified file
        '''
        self.log = Logger(use_syslog, log_file)
        if not root_folder:
            self.root_folder = HWConst.HW_MGMT_FOLDER
        else:
            self.root_folder = root_folder

        self.type = int(thermal_class)
        self.max_tachos = int(max_tachos)
        self.max_psus = int(max_psus)
        self.report_timer = None
        self.polling_timer = None
        self.thermal_table = None
        self.suspend_thermal = '0'
        self.pwm_required = HWConst.PWM_NOACT
        self.fan_dynamic_min = 12
        self.fan_dynamic_min_last = 12
        self.set_cur_state = 0
        self.module_counter = 0
        self.gearbox_counter = 0
        self.periodic_report_time = 0
        self.polling_time = 0

    def __del__(self):
        self.stop()

    def log_set(self, use_syslog = None, log_file = None):
        '''
        @summary:
            Set thermal algorthm logger parameters. Can be called any time
            log provided by /lib/lsb/init-functions always turned on
        @param use_syslog: log also to syslog. Applicable arg value
            1-enable/0-disable
        @param log_file: log to user specified file. Set '' if no log needed
        '''
        res = 0
        if self.log:
            res = self.log.set_param(use_syslog, log_file)
        return res

    def init(self, polling_time = HWConst.THERMAL_POLL_TIME,
             periodic_report_time = HWConst.REPORT_POLL_TIME):
        '''
        @summary:
            Init  thermal algorithm
        @param polling_time: Poll period for thermal analyze
        @param periodic_report_time: Thermal control report interval
        '''
        res = 0
        self.thermal_table = self._get_fan_dynamic_table(self.type)
        if not self.thermal_table:
            self.log.failure("Can't init themal due to unsupported type")
            return 1

        self.module_counter = int(self._read_file('config/module_counter', '0'))
        self.gearbox_counter = int(self._read_file('config/gearbox_counter', '0'))
        self.periodic_report_time = periodic_report_time
        self.polling_time = polling_time

        return res

    def start(self):
        '''
        @summary:
            Start thermal algorithm handling
        '''
        res = 0
        self.log.action('Startnig thermal control service.'
                        'Classs {0}, Tacho:{2}, PSU:{1}'.format(
                                                                self.type,
                                                                self.max_tachos,
                                                                self.max_psus))

        self.log.action("Mellanox thermal control is started")
        delay = int(self._read_file('config/thermal_delay', '0'))
        if delay:
            delay = int(delay)
            time.sleep(delay)

        self.report_timer = RepeatedTimer(self.periodic_report_time, self.thermal_periodic_report)
        if not self.report_timer:
            res = 1
        self.polling_timer = RepeatedTimer(self.polling_time, self.thermal_poll)
        if not self.polling_timer:
            res = 1
        return res

    def stop(self):
        '''
        @summary:
            Stop thermal algorithm handling
        '''
        if self.report_timer:
            self.report_timer.stop()
        if self.polling_timer:
            self.polling_timer.stop()

    def suspend(self):
        '''
        @summary:
            Suspend thermal algorithm
        '''
        self.tz_set_suspend(1)
        self.log.action("Set thermal suspend")

    def resume(self):
        '''
        @summary:
            Resume thermal algorithm
        '''
        self.tz_set_suspend(0)
        self.log.action("Set thermal resume")

    def _read_file(self, filename, default='0'):
        '''
        @summary:
            read file from hw-management tree.
        @param filename: file to read from {hw-management-folder}/filename
        @param default: default value return in case file not exists
        @return: file contents
        '''
        content = ''
        filename = os.path.join(self.root_folder, filename)
        if os.path.isfile(filename):
            with open(filename, 'r') as content_file:
                content = content_file.read().rstrip("\n")
        else:
            content = default
        return content

    def _write_file(self, filename, data):
        '''
        @summary:
            write data to file in hw-management tree.
        @param filename: file to write  {hw-management-folder}/filename
        @param data: data to write
        '''
        filename = os.path.join(self.root_folder, filename)
        try:
            with open(filename, 'w') as content_file:
                content_file.write(str(data))
                content_file.close()
        except IOError:
            pass

    def _thermal_read_file(self, filename, default = '0'):
        '''
        @summary:
            read file from hw-management/thermal tree.
        @param filename: file to read from {hw-management-folder}/thermal/filename
        @param default: default value return in case file not exists
        @return: file contents
        '''
        return self._read_file(os.path.join("thermal", filename), default)

    def _thermal_write_file(self, filename, data):
        '''
        @summary:
            write data to file in hw-management/thermal tree.
        @param filename: file to write  {hw-management-folder}/thermal/filename
        @param data: data to write
        '''
        return self._write_file(os.path.join("thermal", filename), data)

    def _check_file(self, filename):
        '''
        @summary:
            check if file exist in file system in hw-management tree.
        @param filename: file to check {hw-management-folder}/filename
        '''
        filename = os.path.join(self.root_folder, filename)
        return os.path.isfile( filename )

    def _get_fan_dynamic_table(self, thermal_type):
        '''
        @summary:
            get thermal table for the minimum FAN setting per system type
        @param thermal_type: type of thermal system
        '''
        if thermal_type == 1:
            thermal_table = THERMAL_TABLE_LIST["TC1"]
        elif thermal_type == 2:
            thermal_table = THERMAL_TABLE_LIST["TC2"]
        elif thermal_type == 3:
            thermal_table = THERMAL_TABLE_LIST["TC3"]
        elif thermal_type == 4:
            thermal_table = THERMAL_TABLE_LIST["TC4"]
        elif thermal_type == 5:
            thermal_table = THERMAL_TABLE_LIST["TC5"]
        elif thermal_type == 6:
            thermal_table = THERMAL_TABLE_LIST["TC5"]
        else:
            self.log.failure("thermal type {0} is not supported".format(thermal_type))
            return None
        return thermal_table

    def _validate_thermal_configuration(self):
        '''
            Validate FAN fault symbolic links
        '''
        for i in range(1, self.max_tachos + 1):
            if self._thermal_read_file('fan{0}_fault'.format(i)) == '':
                self.log.failure ("FAN fault status attributes are not exist")
                return 1
            elif self._thermal_read_file('fan{0}__speed_get'.format(i)) == '':
                self.log.failure ("FAN input attributes are not exist")
                return 1

        if (self._check_file('thermal/cooling_cur_state') == False or
            self._check_file('thermal/mlxsw/thermal_zone_mode') == False or
            self._check_file('thermal/mlxsw/temp_trip_norm') == False or
            self._check_file('thermal/mlxsw/thermal_zone_temp') == False) :
            self.log.failure ("Thermal zone attributes are not exist")
            return 1

        if (self._check_file('thermal/pwm1') == False or
            self._check_file('thermal/asic') == False) :
            self.log.failure ("PWM control and ASIC attributes are not exist")
            return 1

        for i in range(1, self.module_counter + 1):
            if self._check_file('thermal/module{0}_temp'.format(i)):
                if self._check_file('thermal/module{0}_temp_fault'.format(i)) == False:
                    self.log.failure ("QSFP module attributes are not exist")
                    return 1

        if (self._check_file('thermal/fan_amb') == False or
            self._check_file('thermal/port_amb') == False) :
            self.log.failure ("Ambient temperature sensors attributes are not exist")
            return 1

        if self.max_psus > 0:
            for i in range(1, self.max_psus + 1):
                if self._check_file('thermal/psu{0}_status'.format(i)) == False:
                    self.log.failure ("PS units status attributes are not exist")
                    return 1
        return 0

    def _tz_check_suspend(self):
        '''
        @summary:
            Get suspend state
        @retrn: True if system suspended
        '''
        res = False
        suspend = self._read_file('config/suspend')
        if suspend == '1' :
            res = True
        return res

    def tz_set_suspend(self, suspend=0):
        '''
        @summary:
            Set suspend state
        @tz_name suspend: 1 - susend 0 - resume
        '''
        self._write_file('config/suspend', suspend)

    def _check_untrested_module_sensor(self):
        '''
        @summary:
            Check if some odule if fault stste
        @return: 1 - on sensor failure 0 - Ok
        '''
        untrusted_sensor = 0
        if self._tz_check_suspend() :
            return untrusted_sensor

        for i in range(1, self.module_counter + 1):
            fault = self._thermal_read_file('module{0}_temp_fault'.format(i))
            if fault == '1':
                untrusted_sensor = 1
                break
        return untrusted_sensor

    def _get_tz_val(self, tz_name):
        '''
        @summary:
            Get tz attributes values
        @param tz_name: tx_name
        @return: dict with attribute name:value
        '''
        tz_dict = {}
        tz_dict['temp'] = self._read_file('{0}/thermal_zone_temp'.format(tz_name))
        if tz_dict['temp'] != '':
            tz_dict['mode'] = self._read_file('{0}/thermal_zone_mode'.format(tz_name))
            if tz_dict['mode'] == "enabled":
                tz_dict['tr_norm'] = self._read_file('{0}/temp_trip_norm'.format(tz_name))
                tz_dict['tr_high'] = self._read_file('{0}/temp_trip_high'.format(tz_name))
                tz_dict['tr_hot'] = self._read_file('{0}/temp_trip_hot'.format(tz_name))
                tz_dict['tr_crit'] = self._read_file('{0}/temp_trip_crit'.format(tz_name))
                tz_dict['policy'] = self._read_file('{0}/thermal_zone_policy'.format(tz_name))
                return tz_dict
        return None

    def thermal_periodic_report(self):
        '''
        @summary:
            Periodic thermal status. Call by timer
        '''
        asic_temp = int(self._thermal_read_file('mlxsw/thermal_zone_temp'))
        fan_temp = int(self._thermal_read_file('fan_amb'))
        port_temp = int(self._thermal_read_file('port_amb'))
        cooling = int(self._thermal_read_file('cooling_cur_state'))
        pwm = int(self._thermal_read_file('pwm1'))

        fan_dynamic_state = self.fan_dynamic_min - HWConst.FAN_MAX_STATE
        dyn = fan_dynamic_state * 10

        if cooling > self.set_cur_state:
            self.set_cur_state = cooling
            if cooling > fan_dynamic_state:
                fan_dynamic_state = cooling
        elif cooling >= fan_dynamic_state:
            fan_dynamic_state = cooling
            self.set_cur_state = cooling

        ps_fan_speed = int(PSU_FAN_SPEED[fan_dynamic_state], 16)
        cooling = self.set_cur_state * 10

        self.log.success("Thermal periodic report")
        self.log.success("=======================")
        self.log.success("Temperature(mC): asic {0} fan amb {1} port amb {2}".format(asic_temp, fan_temp, port_temp) )
        self.log.success("Cooling(%): pwm {0} ps_fan_speed {1} dynaimc_min {2}".format(cooling, pwm, ps_fan_speed, dyn) )
        for i in range(1, self.max_tachos + 1):
            tacho = self._thermal_read_file('fan{0}_speed_get'.format(i))
            if tacho:
                fault = self._thermal_read_file('fan{0}_fault'.format(i))
                self.log.success ("tacho{0} speed is {1} fault is {2}".format(i, tacho, fault))

        for i in range(1, self.module_counter + 1):
            temp_input_module = self._thermal_read_file('module{0}_temp_input'.format(i) )
            if temp_input_module :
                if int(temp_input_module) > 0:
                    temp_fault_module = self._thermal_read_file('module{0}_temp_fault'.format(i))
                    temp_crit_module = self._thermal_read_file('module{0}_temp_crit'.format(i))
                    temp_emergency_module = self._thermal_read_file("module{0}_temp_emergency".format(i))
                    self.log.success ( "module{0} temp {1} fault {2} crit {3} emerg {4}".format(i,
                                                                                        temp_input_module,
                                                                                        temp_fault_module,
                                                                                        temp_crit_module,
                                                                                        temp_emergency_module))

                tz_dict = self._get_tz_val('thermal/mlxsw-module{0}'.format(i))
                if tz_dict:
                    self.log.success ("tz module{0} temp {1} trips {2} {3} {4} {5} {6} {7}".format(i,
                                                                                        tz_dict['temp'],
                                                                                        tz_dict['tr_norm'],
                                                                                        tz_dict['tr_high'],
                                                                                        tz_dict['tr_hot'],
                                                                                        tz_dict['tr_crit'],
                                                                                        tz_dict['policy'],
                                                                                        tz_dict['mode']))
        for i in range(1, self.gearbox_counter + 1):
            temp_input_gearbox = self._thermal_read_file('temp_input_gearbox{0}'.format(i))
            if temp_input_gearbox:
                if int(temp_input_gearbox) > 0:
                    self.log.success ( "gearbox{0} temp {1}".format(i, temp_input_gearbox) )
                tz_dict = self._get_tz_val('thermal/mlxsw-gearbox{0}'.format(i))
                if tz_dict:
                    self.log.success ("tz gearbox{0} temp {1} trips {2} {3} {4} {5} {6} {7}".format(i,
                                                                                        tz_dict['temp'],
                                                                                        tz_dict['tr_norm'],
                                                                                        tz_dict['tr_high'],
                                                                                        tz_dict['tr_hot'],
                                                                                        tz_dict['tr_crit'],
                                                                                        tz_dict['policy'],
                                                                                        tz_dict['mode']))
        tz_dict = self._get_tz_val('thermal/mlxsw')
        if tz_dict:
            self.log.success ("tz asic temp {0} trips {1} {2} {3} {4} {5} {6}".format(
                                                                                        tz_dict['temp'],
                                                                                        tz_dict['tr_norm'],
                                                                                        tz_dict['tr_high'],
                                                                                        tz_dict['tr_hot'],
                                                                                        tz_dict['tr_crit'],
                                                                                        tz_dict['policy'],
                                                                                        tz_dict['mode']))

    def _get_psu_presence(self):
        '''
        @summary:
            Get state of system PSU
        @return: pwm value calculated dependig of PSU state
        '''
        res = HWConst.PWM_NOACT
        for psu in range(1, self.max_psus + 1):
            present = self._thermal_read_file("psu{0}_status".format(psu) )
            if present == "0":
                res = HWConst.PWM_MAX
                mode = self._thermal_read_file("mlxsw/thermal_zone_mode")
                if mode == "enabled" :
                    self._thermal_write_file("mlxsw/thermal_zone_mode", "disabled")
                    self.log.action("ASIC thermal zone is disabled due to PS absence")

                self.fan_dynamic_min = HWConst.FAN_DYN_MIN_FAIL
                set_cur_state = self.fan_dynamic_min - HWConst.FAN_MAX_STATE
                self._thermal_write_file("cooling_cur_state", set_cur_state)
                self._thermal_write_file("pwm1", HWConst.PWM_MAX_RPM)
                break
        return res

    def _update_psu_fan_speed(self):
        '''
        @summary:
            Set PSU fan depenging of currnt cooling state
        @return: pwm value calculated dependig of PSU state
        '''
        for psu in range(1, self.max_psus + 1):
            present = self._thermal_read_file("psu{0}_pwr_status".format(psu) )
            if present == "1":
                bus     = self._read_file("config/psu{0}_i2c_bus".format(psu) )
                addr    = self._read_file("config/psu{0}_i2c_addr".format(psu) )
                command = self._read_file("config/fan_command")
                entry   = self._thermal_read_file("cooling_cur_state")
                speed   = PSU_FAN_SPEED[int(entry)]
                subprocess.call("i2cset -f -y {0} {1} {2} {3} wp".format(bus, addr, command, speed), shell = True)

    def _get_fan_faults(self):
        '''
        @summary:
            Get state of system FAN's
        @return: pwm value calculated dependig of FAN state
        '''
        res = HWConst.PWM_NOACT
        for tacho in range(1, self.max_tachos + 1):
            fault = self._thermal_read_file("fan{0}_fault".format(tacho) )
            speed = self._thermal_read_file("fan{0}_speed_get".format(tacho) )
            if fault == '1' or speed == '0':
                res = HWConst.PWM_MAX
                mode = self._thermal_read_file("mlxsw/thermal_zone_mode")
                if mode == "enabled":
                    self._thermal_write_file("mlxsw/thermal_zone_mode", "disabled")
                    self.log.action("ASIC thermal zone is disabled due to FAN fault")

                self.fan_dynamic_min = HWConst.FAN_DYN_MIN_FAIL
                set_cur_state = self.fan_dynamic_min - HWConst.FAN_MAX_STATE
                self._thermal_write_file("cooling_cur_state", set_cur_state)
                self._thermal_write_file("pwm1", HWConst.PWM_MAX_RPM)
                break
        return res

    def _set_pwm_min_threshold(self):
        '''
        @summary:
        '''
        # Check for untrusted modules
        if self._check_untrested_module_sensor():
            trusted = "untrust"
        else:
            trusted = "trust"

        # Define FAN direction
        temp_fan_ambient = int(self._thermal_read_file("fan_amb"))
        temp_port_ambient = int(self._thermal_read_file("port_amb"))

        if temp_fan_ambient > temp_port_ambient:
            ambient = temp_port_ambient
            flow_dir = "p2c"
        elif temp_fan_ambient < temp_port_ambient:
            ambient = temp_fan_ambient
            flow_dir = "c2p"
        else:
            ambient = temp_fan_ambient
            flow_dir = "unk"

        table = self.thermal_table["table"]
        line = table["{0}_{1}".format(flow_dir, trusted)]
        for key, val in line.iteritems():
            t_range = key.split(':')
            t_min = int(t_range[0]) * HWConst.TEMP_TABLE_SCALE
            t_max = int(t_range[1]) * HWConst.TEMP_TABLE_SCALE
            if t_min <= ambient <= t_max:
                self.fan_dynamic_min = val
                break

    def _set_pwm_min_speed(self):
        '''
        @summary:
        '''
        if self._check_untrested_module_sensor():
            self.fan_dynamic_min = self.thermal_table["untrust"]
        else:
            self.fan_dynamic_min = self.thermal_table["trust"]


    def _check_trip_min_vs_current_temp(self):
        '''
        @summary:
        '''
        for module in range(1, self.module_counter + 1):
            temp_now = self._thermal_read_file("mlxsw-module{0}/thermal_zone_temp".format(module))
            if temp_now:
                temp_now = int(temp_now)
                trip_norm = int(self._thermal_read_file("mlxsw-module{0}/temp_trip_norm".format(module)))
                if temp_now > 0 and trip_norm < temp_now:
                    return

        for gearbox in range(1, self.gearbox_counter + 1):
            temp_now = self._thermal_read_file("mlxsw-gearbox{0}/thermal_zone_temp".format(gearbox))
            if temp_now:
                temp_now = int(temp_now)
                trip_norm = int(self._thermal_read_file("mlxsw-gearbox{0}/temp_trip_norm".format(gearbox)))
                if temp_now > 0 and trip_norm < temp_now:
                    return

        temp_now = int(self._thermal_read_file("mlxsw/thermal_zone_temp"))
        trip_norm = int(self._thermal_read_file("mlxsw/temp_trip_norm"))
        if trip_norm > temp_now:
            set_cur_state = self.fan_dynamic_min - HWConst.FAN_MAX_STATE
            self._thermal_write_file("cooling_cur_state", set_cur_state)
            cur_state = set_cur_state*10
            self.log.action("FAN speed is set to {0} percent due to system health recovery".format(cur_state))

    def _disable_zones_def_pwm(self):
        '''
        @summary:
        '''
        mode = self._thermal_read_file("mlxsw/thermal_zone_mode")
        if mode == "enabled":
            self._thermal_write_file("mlxsw/thermal_zone_mode", "disabled")
            self.log.action("ASIC thermal zone is disabled due to thermal algorithm is suspend")

        for module in range(1, self.module_counter + 1):
            mode = self._thermal_read_file("mlxsw-module{0}/thermal_zone_mode".format(module))
            if mode == 'enabled':
                self._thermal_write_file("mlxsw-module{0}/thermal_zone_mode".format(module), "disabled")
                self.log.action("QSFP module {0} thermal zone is disabled due to thermal algorithm is suspend".format(module))

        for gearbox in range(1, self.gearbox_counter + 1):
            mode = self._thermal_read_file("mlxsw-gearbox{0}/thermal_zone_mode".format(gearbox))
            if mode == 'enabled':
                self._thermal_write_file("mlxsw-gearbox{0}/thermal_zone_mode".format(gearbox), "disabled")
                self.log.action("Gearbox {0} thermal zone is disabled due to thermal algorithm is suspend".format(gearbox))

        self._thermal_write_file("pwm1", HWConst.PWM_DEF_RPM)
        self.log.action("Set fan speed to default")

    def thermal_poll(self):
        '''
        @summary:  Main thermal algo loop
        '''
        # Check if thermal is suspended
        suspend = self._read_file("config/suspend")
        if suspend and suspend != self.suspend_thermal:
            if suspend == "1":
                self._disable_zones_def_pwm()
                self.log.action ("Thermal algorithm is manually suspend")
            else:
                self.log.action ("Thermal algorithm is manually resumed")
                time.sleep(1)
            self.suspend_thermal = suspend
            time.sleep(1)
            return
        elif self.suspend_thermal == "1":
            # Validate there is no enabled thermal zones.
            pwm = self._thermal_read_file("pwm1")
            if pwm != HWConst.PWM_DEF_RPM:
                self._disable_zones_def_pwm()
            time.sleep(1)
            return

        if self._validate_thermal_configuration() != 0:
            time.sleep(1)
            return

        # Set PWM minimal limit.
        # Set dynamic FAN speed minimum, depending on ambient temperature,
        # presence of untrusted optical cables or presence of any cables
        # with untrusted temperature sensing.
        self._set_pwm_min_threshold()
        # Update PS unit fan speed
        self._update_psu_fan_speed()
        # If one of PS units is out disable thermal zone and set PWM to the
        # maximum speed.
        if self._get_psu_presence() == HWConst.PWM_MAX:
            return
        # If one of tachometers is faulty disable thermal zone and set PWM
        # to the maximum speed.
        if self._get_fan_faults() == HWConst.PWM_MAX:
            return

        # Update cooling levels of FAN If dynamic minimum has been changed
        # since the last time.
        if self.fan_dynamic_min != self.fan_dynamic_min_last:
            self._thermal_write_file("cooling_cur_state", self.fan_dynamic_min)
            fan_from = (self.fan_dynamic_min_last - HWConst.FAN_MAX_STATE) * 10
            fan_to = (self.fan_dynamic_min - HWConst.FAN_MAX_STATE) * 10
            self.log.action("FAN minimum speed is changed from {0} to {1} percent".format(fan_from, fan_to))
            self.fan_dynamic_min_last = self.fan_dynamic_min
            self._thermal_write_file("fan_dynamic_min", fan_to)

        # Align PWM according to the highest temperature for the cases
        # PWM has been enforced for the particular limit.
        self._check_trip_min_vs_current_temp()


class Daemon(object):
    '''
        @summary:
            Linux Daemon template
    '''
    def __init__(self, pid_file,
                 cmdline_parser_obj,
                 stdout = HWConst.DEVNULL,
                 stderr = HWConst.DEVNULL,
                 ):
        self.stdout = stdout
        self.stderr = stderr
        self.pid_file = pid_file
        self.cmdline_arg = cmdline_parser_obj
        self.thermal_management = None

    def del_pid(self):
        '''
        @summary:
            Delete the pid file
        '''
        os.remove(self.pid_file)

    def daemonize(self):
        '''
        @summary:
            There shined a shiny daemon, In the middle, Of the road...
        '''
        # fork 1 to spin off the child that will spawn the deamon.
        if os.fork():
            sys.exit()

        # This is the child.
        # 1. cd to root for a guarenteed working dir.
        # 2. clear the session id to clear the controlling TTY.
        # 3. set the umask so we have access to all files created by the daemon.
        os.chdir("/")
        os.setsid()
        os.umask(0)

        # fork 2 ensures we can't get a controlling ttd.
        if os.fork():
            sys.exit()

        # This is a child that can't ever have a controlling TTY.
        # Now we shut down stdin and point stdout/stderr at log files.

        # stdin
        with open(HWConst.DEVNULL, 'r') as dev_null:
            os.dup2(dev_null.fileno(), sys.stdin.fileno())

        # stderr - do this before stdout so that errors about setting stdout write to the log file.
        #
        # Exceptions raised after this point will be written to the log file.
        sys.stderr.flush()
        with open(self.stderr, 'a+', 0) as stderr:
            os.dup2(stderr.fileno(), sys.stderr.fileno())

        # stdout
        #
        # Print statements after this step will not work. Use sys.stdout
        # instead.
        sys.stdout.flush()
        with open(self.stdout, 'a+', 0) as stdout:
            os.dup2(stdout.fileno(), sys.stdout.fileno())

        # Write pid file
        # Before file creation, make sure we'll delete the pid file on exit!
        atexit.register(self.del_pid)
        pid = str(os.getpid())
        with open(self.pid_file, 'w+') as pid_file:
            pid_file.write('{0}'.format(pid))

    def get_pid_by_file(self):
        '''
        @summary:
            Return the pid read from the pid file.
        '''
        try:
            with open(self.pid_file, 'r') as pid_file:
                pid = int(pid_file.read().strip())
            return pid
        except IOError:
            return

    def sigterm_handler(self, sig, frame):
        '''
        @summary:
            Signal handler for trination signals
        '''
        print "Thermal-algo: sigterm_handler " + str(os.getpid())
        if self.thermal_management:
            self.thermal_management.log.action("Mellanox thermal control is terminated PID={}".format( os.getpid() ))
            self.thermal_management.stop()
        sys.exit(1)

    def start(self):
        '''
        @summary:
            Starting thermal-controll daemon
        '''
        print "Thermal-algo: starting..."
        pid = self.get_pid_by_file()
        if pid:
            # Only one instance of thermal control could be activated
            if os.path.isdir( os.path.join("/proc", str(pid)) ):
                log = Logger(self.cmdline_arg.use_syslog, self.cmdline_arg.log_file)
                log.warn('Mellanox thermal control is already running.'.format(self.pid_file))
                sys.exit(1)

        if self.cmdline_arg.daemonize == "1":
            self.daemonize()

        signal.signal(signal.SIGTERM, self.sigterm_handler)
        signal.signal(signal.SIGINT,  self.sigterm_handler)

        self.thermal_management = ThermalManagement(self.cmdline_arg.sys_class,
                                                    self.cmdline_arg.tacho,
                                                    self.cmdline_arg.psu,
                                                    root_folder = self.cmdline_arg.root_folder,
                                                    use_syslog=self.cmdline_arg.use_syslog)

        res = self.thermal_management.log_set(log_file=self.cmdline_arg.log_file)
        if res != 0:
            sys.exit(1)
        if self.thermal_management.init(HWConst.THERMAL_POLL_TIME, HWConst.REPORT_POLL_TIME) != 0:
            sys.exit(1)

        if self.cmdline_arg.wait:
            time.sleep(int(self.cmdline_arg.wait))

        if  self.thermal_management.start() != 0:
            self.thermal_management.stop()
            sys.exit(1)

         #The main loop of the daemon.
        while 1:
            time.sleep(1)

    def stop(self):
        '''
        @summary:
            Stop the thermal-controll daemon.
        '''
        print "Thermal-algo: stopping..."
        pid = self.get_pid_by_file()
        if not pid:
            print "Err. PID file {0} doesn't exist. Is the Mellanox thermal control running?".format(self.pid_file)
            return

        # Time to kill
        try:
            time_end = time.time() + 10
            while time.time() < time_end:
                os.kill(pid, signal.SIGTERM)
                time.sleep(0.1)
            os.kill(pid, signal.SIGKILL)
        except OSError as err:
            if 'No such process' in err.strerror and os.path.exists(self.pid_file):
                os.remove(self.pid_file)

    def restart(self):
        '''
        @summary:
            Restart the deamon.
        '''
        self.stop()
        self.start()

    def suspend(self):
        '''
        @summary:
            Set suspend flag to hw-management algo
        '''
        self.thermal_management = ThermalManagement(self.cmdline_arg.sys_class,
                                                    self.cmdline_arg.tacho,
                                                    self.cmdline_arg.psu,
                                                    root_folder = self.cmdline_arg.root_folder)

        res = self.thermal_management.log_set(use_syslog=self.cmdline_arg.use_syslog,
                                        log_file=self.cmdline_arg.log_file)

        if res != 0:
            sys.exit(1)
        self.thermal_management.suspend()

    def resume(self):
        '''
        @summary:
            Set suspend flag to hw-management algo
        '''
        self.thermal_management = ThermalManagement(self.cmdline_arg.sys_class,
                                                    self.cmdline_arg.tacho,
                                                    self.cmdline_arg.psu,
                                                    root_folder = self.cmdline_arg.root_folder)

        res = self.thermal_management.log_set(use_syslog=self.cmdline_arg.use_syslog,
                                        log_file=self.cmdline_arg.log_file)
        if res != 0:
            sys.exit(1)
        self.thermal_management.resume()

    def status(self):
        '''
        @summary:
            Get servise status 1- means active
        '''
        pid = self.get_pid_by_file()
        if pid:
            if os.path.isdir( os.path.join("/proc", str(pid)) ):
                print "service is running"
                return 0
        print "service stopped"
        return 0

if __name__ == '__main__':
    try:
        CMD_PARSER = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
        CMD_PARSER.add_argument('cmd',
                            choices=('start', 'stop', 'restart', 'suspend', 'resume', "status"),
                            help='Mandatory - command start|stop|restart|suspend|resume|status of thermal control',
                            nargs='?')
        CMD_PARSER.add_argument('-c', '--class',
                            dest='sys_class',
                            help='Define system thermal class depending of system type',
                            default=1)
        CMD_PARSER.add_argument('-t', '--max_tacho',
                            dest='tacho',
                            help='Define FAN Tacho count for current system',
                            default=4)
        CMD_PARSER.add_argument('-p', '--max_psu',
                            dest='psu',
                            help='Define replaseble PSU count for current system',
                            default=0)
        CMD_PARSER.add_argument('-f', '--config-file',
                            dest='config_file',
                            help='load options from config file f.e. /tmp/thermal_control_conf',
                            default=HWConst.CONFIG_FILE)
        CMD_PARSER.add_argument('-l', '--log_file',
                            dest='log_file',
                            help='add output to log file',
                            default=None)
        CMD_PARSER.add_argument('-s', '--syslog',
                            dest='use_syslog',
                            help='enable(1)/disable(0) output to syslog',
                            default='0',
                            choices=('0', '1'))
        CMD_PARSER.add_argument('-r', '--root_folder',
                            dest='root_folder',
                            help='Define hw-managewment folder for sensor reading',
                            default=HWConst.HW_MGMT_FOLDER)
        CMD_PARSER.add_argument('-w', '--wait',
                            dest='wait',
                            help='Define how mant sec wait before start algo',
                            default=HWConst.WAIT_START)
        CMD_PARSER.add_argument('-d', '--daemonize',
                            dest='daemonize',
                            help='Rus as a deamon',
                            default=0)

        CMD_PARSER_ARGS = CMD_PARSER.parse_args()

        if os.path.isfile(CMD_PARSER_ARGS.config_file):
            with open(CMD_PARSER_ARGS.config_file, 'r') as config:
                CMD_PARSER_ARGS = CMD_PARSER.parse_args(config.read().split() + sys.argv[1:])

        DAEMON = Daemon(HWConst.HW_MGMT_PID, CMD_PARSER_ARGS)

        if CMD_PARSER_ARGS.cmd == 'start':
            DAEMON.start()
        elif CMD_PARSER_ARGS.cmd == 'stop':
            DAEMON.stop()
        elif CMD_PARSER_ARGS.cmd == 'restart':
            DAEMON.restart()
        elif CMD_PARSER_ARGS.cmd == 'suspend':
            DAEMON.suspend()
        elif CMD_PARSER_ARGS.cmd == 'resume':
            DAEMON.resume()
        elif CMD_PARSER_ARGS.cmd == 'status':
            DAEMON.status()
        else:
            print "Inavlid cmd {0}".format(CMD_PARSER_ARGS.cmd)
            sys.exit(1)

    except Exception as err:
        traceback.print_exc()
        print err.message

    sys.exit(0)
