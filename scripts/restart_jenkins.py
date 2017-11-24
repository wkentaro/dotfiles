#!/usr/bin/env python

from __future__ import print_function

import os.path as osp
import time

from selenium.common.exceptions import ElementNotInteractableException
from selenium.common.exceptions import NoAlertPresentException
from selenium.common.exceptions import NoSuchElementException
from selenium.common.exceptions import NoSuchFrameException
from selenium import webdriver


print('==> Opening browser.')
driver = webdriver.Firefox(
    executable_path=osp.expanduser('~/.local/bin/geckodriver'),
    log_path='/tmp/restart_jenkins_py_geckodriver.log')
driver.get('https://133.11.216.231/login.html')
assert 'idrac-B1FKWBX' in driver.title
print('==> Complete.')

print('==> Logging in Jenkins server controller.')
input_user = driver.find_element_by_id('user')
input_user.send_keys('admin')
input_password = driver.find_element_by_id('password')
input_password.send_keys('.e6Hou')
login_button = driver.find_element_by_id('btnOK')
login_button.click()
print('==> Complete.')

print('==> Waiting for loading webpage.')
while True:
    try:
        driver.switch_to_frame('da')
        break
    except NoSuchFrameException:
        time.sleep(0.1)
print('==> Complete.')

print('==> Pushing button of Power Cycle System (cold boot).')
while True:
    try:
        button_pwrcycle = driver.find_element_by_id('pwrcycle')
    except NoSuchElementException:
        continue

    try:
        button_pwrcycle.click()
    except ElementNotInteractableException:
        continue

    try:
        driver.switch_to_alert().accept()
    except NoAlertPresentException:
        continue

    break
print('==> Complete.')

print('==> Closing browser.')
driver.close()
print('==> Complete.')
