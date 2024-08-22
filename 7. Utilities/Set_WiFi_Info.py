import requests
import base64
import json
import time


def put_wireless(ssid, passwd):
    '''
    Set WiFi name and password
    Note: both "ssid" and "passwd" are strings. And you can have numbers, alphabets, underscore, and hyphen in "ssid". And "passwd" needs to have at least eight characters.
    '''
    headers = {'Content-Type': 'application/json;charset=UTF-8'}
    d = {
        "ssid": ssid,
        "passwd": passwd
    }
    url = 'http://192.168.12.1:8080/wireless'
    res = requests.put(url, data=json.dumps(d), headers=headers)
    print(res)

put_wireless("Wifi_name_here","12345678")