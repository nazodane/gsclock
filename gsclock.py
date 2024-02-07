#!/usr/bin/python
# Copyright (C) 2024 Toshimitsu Kimura <lovesyao@gmail.com>
# SPDX-License-Identifier: BSD-3-Clause
from time import sleep
import requests
import re
import os

CLOCK_MONITOR = "HDMI-1-1" # you should specify vertical monitor

print("QUERY: clock monitor check via screeninfo module")
clock_monitor = None
# pip install screeninfo
from screeninfo import get_monitors
for m in get_monitors():
    if m.name == CLOCK_MONITOR:
        print("Found: " + str(m))
        clock_monitor = m
        break
    print("Ignore: " + str(m))

if not clock_monitor:
    print("FAILED: no clock monitor found!")
    exit()

print("SUCCESS: clock monitor found!")


# 他にユーザーの居るシリアルコンソールは基本的に奪わないようにする
# シリアルを繋いでみる→目的のデータが飛んでるならシリアルナンバーを保存→シリアルナンバーを保存したものだけfuser -kを飛ばす

import subprocess
def find_processes_using_dev(dev_name): # psutil not works.
    try:
        output = subprocess.check_output(["fuser", dev_name])
        pids = output.decode().split()
        return [int(pid) for pid in pids]
    except subprocess.CalledProcessError:
        return []

print("QUERY: arduino serial port check via pyudev module, pyserial module and psmisc's fuser")
# pip install pyserial pyudev
import serial
import pyudev
udevc = pyudev.Context()

arduino = None
for device in udevc.list_devices(subsystem='tty'):
  if 'ID_VENDOR_ID' not in device.properties or 'ID_MODEL_ID' not in device.properties: # if !isUSB
    continue
  port = device.device_node
  vendor_id = device.properties['ID_VENDOR_ID']
  model_id = device.properties['ID_MODEL_ID']
  serial_number = device.properties.get("ID_SERIAL") # or ID_SERIAL_SHORT
  print("candidate of the arduino serial port: %s %s:%s %s"%(port, vendor_id, model_id, serial_number))

  count = 3
  while count > 0:
    count -= 1
    try:
      print("checking the serial port %s..."%(port))
      pids = find_processes_using_dev(port)
      if len(pids): # if the port is used, we shouldn't steal the input of the port.
          print("used by pids %s so skipping the serial port %s..."%(pids, port))
          # if the serial number is in whitelist, use this anyways. -> not implemented
          # os.system("fuser -k %s"%(port))
          count = 0
          continue
      print("connecting %s..."%(port))
      arduino = serial.Serial(port, 9600, timeout=1)
      arduino.flush()
      arduino.read_until().decode('utf-8').rstrip() # drop the garbage input
      line = arduino.read_until().decode('utf-8').rstrip()
      print(line)
      if line.startswith(("praw:", "temp:", "humi:")):
          break
      arduino.close()
      arduino = None
      print("failed and closing the serial port %s..."%(port))
    except KeyboardInterrupt:
      print("Keybord interrupt, exiting...")
      if arduino:
        arduino.close()
      exit()
    except:
      print("WARNING: error to connect the arduino serial, retrying...")

if not arduino:
    print("FAILED: no arduino serial port found!")
    exit()

print("SUCCESS: arduino serial port found!")


CLOCK_SIZE = 1080

if __name__ == '__main__':
    # pip install selenium
    from selenium import webdriver
    from selenium.webdriver.chrome.options import Options
    from selenium.webdriver.chrome import service as fs
    from selenium.common.exceptions import WebDriverException
    from selenium.webdriver.common.by import By
    print("QUERY: chrome window opening via selenium module")
    chrome_options = Options()
    chrome_options.add_experimental_option("excludeSwitches", ['enable-automation'])
    chrome_options.add_argument("--app=file://%s/gsclock.html"%(os.getcwd()))
    chrome_options.add_argument("--window-position=%s,%s"%(m.x, m.y))
    chrome_options.add_argument("--start-fullscreen")
    chrome_service = fs.Service(executable_path="/usr/lib/chromium-browser/chromedriver")
    driver = webdriver.Chrome(service=chrome_service , options=chrome_options)
    print("SUCCESS: chrome window opened")

    print("QUERY: chrome window stickying via wmctrl")
    # pip install wmctrl
    from wmctrl import Window

    title_bak = driver.execute_script("return document.title;")
    det_title = "window id detection by clock.py"
    driver.execute_script("document.title = '%s';"%(det_title))
    sleep(1) # XXX: 0.1 not works...
    window = [w for w in Window.list() if w.wm_name == det_title][0]
    window.set_properties(["add", "sticky"])
    driver.execute_script("document.title = '%s';"%(title_bak))

    print("SUCCESS: chrome window stickied")

    while True:
        try:
#            line = arduino.readline().decode('utf-8').rstrip()
            line = arduino.read_until().decode('utf-8').rstrip()
            print(line)
            elm = driver.find_element(By.ID, line[0:4])
            driver.execute_script("arguments[0].innerText = arguments[1]", elm, line[6:])
        except KeyboardInterrupt:
            print("Keybord interrupt, exiting...")
            exit()
        except:
            print("TODO")
            pass

# TODO: chromeウインドウを閉じたら閉じる

