#!/usr/bin/python

import pytz
import urllib
import icalendar
from datetime import datetime
import dateutil.rrule
import logging

logging.basicConfig(level=logging.DEBUG)
default_temp = 273+21
url = "https://www.google.com/calendar/ical/rbudok4qefqrjembvdba686n7s%40group.calendar.google.com/private-a967fd441486b8f9a75325827ae00778/basic.ics"

def fetch_calendar():
        f = urllib.urlopen(url)
        return icalendar.Calendar.from_string(f.read())

def parse_temp(temp):
        """ Parse a temperature and convert to degrees Kelvin """
        filtered = filter(lambda c: not unicode.isalpha(c), temp)
        try:
                if temp.endswith('C'):
                        return float(filtered) + 273
                elif temp.endswith('F'):
                        return (float(filtered) - 32) * 5/9 + 273
                else:
                        return None
        except Exception as e:
                logging.error('Error parsing temperature "%s"' % temp)
                return None

def event_happening(ev):
        start = ev['DTSTART'].dt
        end = ev['DTEND'].dt
        length = end - start
        if ev.has_key('RRULE'):
                rr = dateutil.rrule.rrulestr(str(ev['RRULE']), dtstart=ev['DTSTART'].dt)
                for a in rr.between(datetime.now() - length, datetime.now()):
                        return True
                return False
        else:
                return datetime.now(pytz.utc) > start and datetime.now(pytz.utc) < end
                
def update_setpoint():
        cal = fetch_calendar()
        events = filter(lambda ev: type(ev) is icalendar.Event
                               and parse_temp(ev['SUMMARY']) is not None
                               and event_happening(ev), cal.subcomponents)
        events = sorted(events, key=lambda ev: ev['DTEND'].dt - ev['DTSTART'].dt)
        if len(events) > 0:
                t = parse_temp(events[0]['SUMMARY'])
                logging.debug("Setpoint from event '%s' = %f" % (events[0]['SUMMARY'], t))
                return t
        else:
                logging.debug("Default setpoint = %f" % default_temp)
                return default_temp


if __name__ == '__main__':
        from time import time, sleep
        import serial
        s = serial.Serial('/dev/ttyUSB0')
        s.write('no echo\n')
        sleep(0.1)
        s.flushInput()

        setpoint = None
        log = open('temperature.log', 'a')
        while True:
                setpoint = update_setpoint()
                logging.info("Setpoint: %f" % setpoint)
                s.write('set target %f\n' % setpoint)
                s.readline()

                s.write('temp\n')
                temp = float(s.readline().split()[1])
                log.write("%d\t%f\t%f\n" % (time(), temp, setpoint))

                sleep(2*60)

