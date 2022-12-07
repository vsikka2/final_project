
import picar_4wd as fc
import numpy as np
import sys
import heapq
from time import sleep
import requests

TELEGRAM_SENDER_API_TOKEN = '5842961554:AAE9wr_AZxrM2EongXw8T7ayEsvzptmLmC0'

def send_to_telegram(message):

    apiToken = TELEGRAM_SENDER_API_TOKEN
    chatID = '246452512'
    apiURL = f'https://api.telegram.org/bot{apiToken}/sendMessage'

    try:
        response = requests.post(apiURL, json={'chat_id': chatID, 'text': message})
    except Exception as e:
        a=1


def main():
    while True:
        dist = fc.get_distance_at(90)
        if(dist!=-2):
            send_to_telegram("Movement detected")
            sleep(5)

        
if __name__ == "__main__":
    try: 
        main()
    except: 
        a=1
        
