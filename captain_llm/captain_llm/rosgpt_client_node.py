#!/usr/bin/env python3
# This file is part of rosgpt package.
#
# Copyright (c) 2023 Anis Koubaa.
# All rights reserved.
#
# This work is licensed under the terms of the Creative Commons Attribution-NonCommercial-ShareAlike 4.0
# International Public License. See https://creativecommons.org/licenses/by-nc-sa/4.0/ for details.

## Flask 서버(/rosgpt)에 HTTP POST를 보내서 LLM 응답을 받아 콘솔에 찍는 CLI형 ROS2 노드
## ROS 토픽 퍼블리시/구독은 하지 않고, 사용자 입력 → HTTP 요청 → 응답 파싱만 담당해.

import json
import rclpy
from rclpy.node import Node
import requests
import uuid


class ROSGPTClient(Node):
    def __init__(self):
        super().__init__('rosgpt_client')
        self.declare_parameter('server_url', 'http://localhost:5000/rosgpt')
        self.server_url = self.get_parameter('server_url').value

        self.get_logger().info('ROSGPT client node started')

        self.client_id = str(uuid.uuid4())  # 실행 인스턴스마다 고유 ID

        ## 바로 send_text_command 호출
        self.send_text_command()

    # def send_text_command(self):
    #     """
    #     Sends a text command to the ROSGPT system and receives a response from the ChatGPT language model.
    #     """
    #     while rclpy.ok():
    #         print('Enter a move command or a rotate command. The current ROSGPTParser of rosgpt_turtlesim does not multiple command. Will be extended later')
    #         text_command = input("Enter a text command: ")
    #         data = {'text_command': text_command}

    #         response = requests.post(self.server_url, data=data)

    #         if response.status_code == 200:
    #             try:
    #                 response_str = response.content.decode('utf-8')
    #                 response_dict = json.loads(response_str)

    #                 self.get_logger().info('Response: {}'.format(response_dict['text']))
    #                 self.get_logger().info('JSON: {}'.format(json.loads(response_dict['json'])))
    #             except Exception as e:
    #                 print('[Exception] An unexpected error occurred:', str(e)) 
    #         else:
    #             self.get_logger().error('Error: {}'.format(response.status_code))

    
    def send_text_command(self):
        while rclpy.ok():
            print('Enter a move command or a rotate command. ...')
            text_command = input("Enter a text command: ")

            # 여기: client_id 같이 보냄
            data = {'text_command': text_command, 'client_id': self.client_id}

            # (권장) 타임아웃도 추가
            response = requests.post(self.server_url, data=data, timeout=15)

            if response.status_code == 200:
                try:
                    response_str = response.content.decode('utf-8')
                    response_dict = json.loads(response_str)
                    self.get_logger().info('Response: {}'.format(response_dict['text']))
                    self.get_logger().info('JSON: {}'.format(json.loads(response_dict['json'])))
                except Exception as e:
                    print('[Exception] An unexpected error occurred:', str(e))
            else:
                self.get_logger().error('Error: {}'.format(response.status_code))


def main(args=None):
    rclpy.init(args=args)

    rosgpt_client = ROSGPTClient()

    rclpy.spin(rosgpt_client)

    rosgpt_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
