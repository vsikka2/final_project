
import picar_4wd as fc
import numpy as np
import sys
import heapq
from time import sleep
import requests

TELEGRAM_SENDER_API_TOKEN = ''

# def send_to_telegram(message):

#     apiToken = TELEGRAM_SENDER_API_TOKEN
#     chatID = '515382482'
#     apiURL = f'https://api.telegram.org/bot{apiToken}/sendMessage'

#     try:
#         response = requests.post(apiURL, json={'chat_id': chatID, 'text': message})
#         print(response.text)
#     except Exception as e:
#         print(e)


def main():
    while True:
        dist = fc.get_distance_at(90)
        if(dist!=-2):
            print(dist)
            send_to_telegram("message")
        sleep(5)

        
if __name__ == "__main__":
    try: 
        main()
    finally: 
        fc.stop()
