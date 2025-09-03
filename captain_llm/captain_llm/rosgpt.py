#!/usr/bin/env python3
# This file is part of rosgpt package.
#
# Copyright (c) 2023 Anis Koubaa.
# All rights reserved.
#
# This work is licensed under the terms of the Creative Commons Attribution-NonCommercial-ShareAlike 4.0
# International Public License. See https://creativecommons.org/licenses/by-nc-sa/4.0/ for details.

# 역할: Flask 웹서버(REST) + ROS2 노드(퍼블리셔) + OpenAI LLM + TTS를 한 파일에 묶은 “게이트웨이”.

# 입력: HTTP POST /rosgpt (form 필드 text_command).

# 처리: 프롬프트 생성 → OpenAI Chat API 호출 → 응답에서 JSON 부분 추출.

# 출력: voice_cmd(std_msgs/String) 토픽으로 LLM 응답 전체를 문자열로 퍼블리시.

# 부가: /에 webapp의 index.html 서빙, pyttsx3로 음성 알림.

import os
import json
import openai
import rclpy
import threading
from rclpy.node import Node
from std_msgs.msg import String
from flask import Flask, request, send_from_directory, jsonify
from flask_restful import Resource, Api
from flask_cors import CORS
import pyttsx3  # pip install pyttsx3 #you need to install libespeak1 on Ubuntu # sudo apt-get install libespeak1
from rclpy.executors import SingleThreadedExecutor
import subprocess

import threading
from collections import defaultdict

from ament_index_python import get_package_share_directory

import re

# 캡틴 온톨로지: 고정 매핑 (LLM이 action으로 내더라도 최종적으로 이 표로 정규화)
# === 상태: 시작한 client_id 집합 ===
started_clients = set()

ACTION_TO_ORDERS = {

    "check_navigation": [{"order_id": 4, "detail": "navigation check"}],
    "nav_check":        [{"order_id": 4, "detail": "navigation check"}],
    "check_thruster":   [{"order_id": 3, "detail": "thruster check"}],
    "propulsor_check":  [{"order_id": 3, "detail": "thruster check"}],
    "check_ship_status":[{"order_id": 3, "detail": "thruster check"},
                         {"order_id": 4, "detail": "navigation check"}],
    "inspection":       [{"order_id": 3, "detail": "thruster check"},
                         {"order_id": 4, "detail": "navigation check"}],
    "depart":           [{"order_id": 5, "detail": "departure"}],
    "sail":             [{"order_id": 5, "detail": "departure"}],
    "mission_start":    [],  # orders는 캡틴이 시작만 처리하므로 빈 리스트
}

SYSTEM_PROMPT = """You are a command normalizer for a maritime USV "AI Captain".
Return a SINGLE JSON object ONLY (no extra text). Do not include code fences.
Schema (examples):

Input: "Mission Start"
Output:
{"action":"mission_start"}

Input: "Check Ship Navigation system"
Output:
{"action":"check_navigation"}

Input: "선박 상태 검사해줘"
Output:
{"action":"check_ship_status"}

Input: "추진기 검사해줘"
Output:
{"action":"check_thruster"}

Input: "선박 출항하자"
Output:
{"action":"depart"}

Rules:
- Respond with exactly one JSON object.
- The "action" must be one of:
  ["mission_start","check_navigation","nav_check","check_thruster","propulsor_check",
   "check_ship_status","inspection","depart","sail"].
"""

def extract_json_only(text: str) -> str:
    """가장 그럴듯한 JSON 객체 하나를 문자열로 추출 (```json``` 블록 우선, 없으면 {..} 범위)."""
    # ```json ... ```
    m = re.search(r"```json\s*(\{.*?\})\s*```", text, re.DOTALL | re.IGNORECASE)
    if m:
        return m.group(1).strip()
    # 첫 { ~ 마지막 } 범위
    start = text.find('{')
    end = text.rfind('}')
    if start != -1 and end != -1 and end > start:
        return text[start:end+1].strip()
    return ""  # 없으면 빈 문자열


history_lock = threading.Lock()
# {client_id: [{"role":"system","content":...}, {"role":"user","content":...}, {"role":"assistant","content":...}, ...]}
chat_histories = defaultdict(list)

MAX_TURNS = 6  # 사용자-어시스턴트 묶음을 몇 개 유지할지(필요에 맞게 조정)

# Instantiate a Flask application object with the given name
app = Flask(__name__)

# Enable Cross-Origin Resource Sharing (CORS) for the Flask app
CORS(app)

# Create an API object that wraps the Flask app to handle RESTful requests
api = Api(app)

#You must add OPENAI_API_KEY as an environment variable
#In Ubuntu: echo 'export OPENAI_API_KEY=your_api_key' >> ~/.bashrc
# Get the API key from the environment variable. 
openai_api_key = os.getenv('OPENAI_API_KEY')
#print(openai_api_key)

# Now you can use the openai_api_key variable to authenticate with the OpenAI API


# Initialize a threading lock for synchronizing access to shared resources
# when multiple threads are involved
spin_lock = threading.Lock()

# Initialize the Text-to-Speech (TTS) engine using the pyttsx3 library
# you need to install the following dependencies 
#       sudo apt-get install libespeak1
#       pip3 install pyttsx3
tts_engine = pyttsx3.init()

# Create a separate threading lock for synchronizing access to the TTS engine
tts_lock = threading.Lock() 

def build_messages(client_id: str, new_user_content: str):
    with history_lock:
        hist = chat_histories[client_id]
        if not hist:
            # 대화 시작 시 1회만 system 프롬프트 삽입
            hist.append({"role": "system", "content": "You are a helpful assistant."})
        # 새 user 턴 추가
        hist.append({"role": "user", "content": new_user_content})
        # 최근 MAX_TURNS*2 + 1(system)만 남기기
        # system(1) + (user,assistant)*k 구조 가정
        if len(hist) > 1 + MAX_TURNS*2:
            # system은 유지하고 뒤에서부터 자르기
            keep = [hist[0]] + hist[-MAX_TURNS*2:]
            chat_histories[client_id] = keep
            hist = keep
        return list(hist)  # shallow copy

def record_assistant_reply(client_id: str, assistant_text: str):
    with history_lock:
        chat_histories[client_id].append({"role": "assistant", "content": assistant_text})


def speak(text):
    # pyttsx3 TTS 호출 래퍼.
    # --> 같은 엔진 인스턴스에 여러 스레드가 말걸면 꼬일 수 있다고 함. 요청 수신/응답 수신 시 비동기로 안내 멘트 발화.
    """
    This function uses the Text-to-Speech (TTS) engine to speak the given text.

    It is an optional method that can be used if you want the system to audibly
    communicate the text messages.

    Args:
        text (str): The text to be spoken by the TTS engine.

    Note:
        This method is optional and can be used when audible communication of text
        messages is desired. If not needed, it can be omitted from the implementation.
    """
    # Acquire the TTS lock to ensure exclusive access to the TTS engine
    with tts_lock:
        # Instruct the TTS engine to say the given text
        tts_engine.say(text)

        # Block and wait for the TTS engine to finish speaking
        tts_engine.runAndWait()



class ROSGPTNode(Node):
    ## 역할: ROS2 노드(이름: chatgpt_ros2_node) + 퍼블리셔 1개 생성.
    
    # 퍼블리셔: 토픽 voice_cmd, 메시지 타입 std_msgs/String, 큐 사이즈 10.

    # publish_message(message): 들어온 문자열을 String.data에 넣어 퍼블리시.

    def __init__(self):
        """
        Initialize the ROSGPTNode class which is derived from the rclpy Node class.
        """
        # Call the superclass constructor and pass the name of the node
        super().__init__('chatgpt_ros2_node')
        # Create a publisher for the 'voice_cmd' topic with a message queue size of 10
        self.publisher = self.create_publisher(String, 'voice_cmd', 10)

    def publish_message(self, message):
        """
        Publish the given message to the 'voice_cmd' topic.
        Args:
            message (str): The message to be published.
        """
        msg = String() # Create a new String message 
        msg.data = message # Convert the message to a JSON string and set the data field of the message
        self.publisher.publish(msg) # Publish the message using the publisher 
        #print('message Published: ', message) # Log the published message
        #print('msg.data Published: ', msg.data) # Log the published message
        
        



def process_and_publish_chatgpt_response(chatgpt_ros2_node, text_command, chatgpt_response, use_executors=True):
    ## 무엇: ROSGPTNode.publish_message() 호출 + “스핀 한 번”.
    """
    Process the chatbot's response and publish it to the 'voice_cmd' topic.

    Args:
        chatgpt_ros2_node (ROSGPTNode): The ROS2 node instance.
        text_command (str): The text command received from the user.
        chatgpt_response (str): The response from the chatbot.
        use_executors (bool, optional): Flag to indicate whether to use SingleThreadedExecutor. Defaults to True.
    """
    chatgpt_ros2_node.publish_message(chatgpt_response) # Publish the chatbot's response using the ROS2 node
    # If use_executors flag is True, use SingleThreadedExecutor
    # 의도: Flask(WSGI) 스레드에서 ROS 스핀을 안전하게 한 번 돌려 이벤트 큐를 처리하려는 용도.
    if use_executors:
        # 매 요청마다 SingleThreadedExecutor를 새로 만들고 노드를 붙였다가

        executor = SingleThreadedExecutor()# Create a new executor for each request 
        executor.add_node(chatgpt_ros2_node) # Add the node to the executor
        executor.spin_once()#  Spin the executor once

        # spin_once()만 실행 후 제거.

        executor.remove_node(chatgpt_ros2_node) # Remove the node from the executor
    # If use_executors flag is False, use spin_lock to synchronize access
    else:
        with spin_lock:
            rclpy.spin_once(chatgpt_ros2_node)



class ROSGPTProxy(Resource):
    # 이 클래스가 /rosgpt 엔드포인트를 담당합니다.
    """
    A class derived from flask_restful.Resource, responsible for handling incoming HTTP POST requests.
    """

    def __init__(self, chatgpt_ros2_node):

        # Flask 리소스가 ROS 노드를 참조할 수 있게 주입.
        """
        Initialize the ROSGPTProxy class with the given ROS2 node.

        Args:
            chatgpt_ros2_node (ROSGPTNode): The ROS2 node instance.
        """
        self.chatgpt_ros2_node = chatgpt_ros2_node

    def askGPT(self, text_command, client_id):
        # 1) system 프롬프트 강제 + user는 원문
        new_user_content = text_command.strip()
        messages = build_messages(client_id, new_user_content)
        if messages and messages[0].get("role") == "system":
            messages[0]["content"] = SYSTEM_PROMPT
        else:
            messages.insert(0, {"role": "system", "content": SYSTEM_PROMPT})

        # 2) 호출
        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo", messages=messages, temperature=0.0
            )
        except openai.error.InvalidRequestError as e:
            print(f"Error: {e}"); return None
        except Exception as e:
            print(f"Unexpected error: {e}"); return None

        raw = response.choices[0].message['content'].strip()
        record_assistant_reply(client_id, raw)

        # 3) JSON 추출/파싱
        jtxt = extract_json_only(raw)
        final_json = {"mission": "none", "orders": [], "note": ""}

        # 시작 여부
        started = (client_id in started_clients)

        if not jtxt:
            # JSON이 아니면 아예 의도 해석 안 함 (캡틴이 무시)
            final_json["note"] = "no json; ignored"
        else:
            try:
                j = json.loads(jtxt)
                action = str(j.get("action","")).lower()

                # 미션 전에는 mission_start만 허용
                if not started:
                    if action == "mission_start":
                        final_json["mission"] = "start"
                        final_json["note"] = "mission started"
                        started_clients.add(client_id)
                    else:
                        final_json["mission"] = "not_started"
                        final_json["note"] = "mission not started; only mission_start allowed"
                else:
                    # 이미 시작한 경우: mission은 none, orders만 허용
                    if action in ACTION_TO_ORDERS:
                        if action == "mission_start":
                            final_json["mission"] = "none"
                            final_json["note"] = "already started; ignored mission_start"
                        else:
                            final_json["orders"] = ACTION_TO_ORDERS[action]
                            final_json["note"] = f"parsed action={action}"
                    else:
                        final_json["note"] = f"unknown action: {action}"
            except Exception as e:
                final_json["note"] = f"json parse error; {e}"

        # 4) 퍼블리시용 페이로드
        payload = json.dumps({
            "text": text_command,
            "json": json.dumps(final_json, ensure_ascii=False)
        }, ensure_ascii=False)

        return payload


    # 이 post()는 Flask 서버에서 외부 요청에 응답할 때 자동 호출되는 거지, 코드 내부에서 직접 호출되는 게 아니야. 라고함? --> Flask post()는 서버 핸들러
    def post(self):
        """
        Handles an incoming POST request containing a text command. The method sends the text command
        to the GPT-3 model and processes the response using the process_and_publish_chatgpt_response function in a separate thread.
        
        Returns:
            dict: A dictionary containing the GPT-3 model response as a JSON string.
            
        텍스트 명령이 포함된 수신 POST 요청을 처리합니다. 이 메서드는 텍스트 명령을 
        GPT-3 모델로 전송하고 별도의 스레드에서 process_and_publish_chatgpt_response 함수를 사용하여 응답을 처리합니다.

        반환값:
        dict: GPT-3 모델 응답을 JSON 문자열로 포함하는 사전입니다.
        """

        client_id = request.form.get('client_id', 'default')  # client 구분 없으면 전원 공유 주의!

        text_command = request.form['text_command']

        print("\n")
        print ('[ROSGPT] Command received. ', text_command, '. Asking ChatGPT ...')
        print("\n")

        # Run the speak function on a separate thread
        #print('text_command:', text_command,'\n')
        threading.Thread(target=speak, args=(text_command+"Message received. Now consulting ChatGPT for a response.",)).start()

        # LLM 호출: chatgpt_response = self.askGPT(text_command)
        # chatgpt_response = self.askGPT(text_command)
        chatgpt_response = self.askGPT(text_command, client_id)

        # print ('[ROSGPT] Response received from ChatGPT. \n', str(json.loads(chatgpt_response))[:60], '...')
        
        #print('eval(chatgpt_response)', eval(chatgpt_response))
        # Run the speak function on a separate thread
        threading.Thread(target=speak, args=("We have received a response from ChatGPT.",)).start()

        if chatgpt_response is None:
            return {'error': 'An error occurred while processing the request'}

        threading.Thread(target=process_and_publish_chatgpt_response, args=(self.chatgpt_ros2_node, text_command, chatgpt_response, True)).start()
        #print(json.loads(chatgpt_response))
        return json.loads(chatgpt_response)


@app.route('/')
def index():
    #print(os.path.join(app.root_path, 'webapp'))
    return send_from_directory(os.path.join(get_package_share_directory('rosgpt'), 'webapp'), 'index.html')


def main():
    rclpy.init(args=None)
    chatgpt_ros2_node = ROSGPTNode()
    api.add_resource(ROSGPTProxy, '/rosgpt', resource_class_args=(chatgpt_ros2_node,))
    app.run(debug=True, host='0.0.0.0', port=5000)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


"""

서버 측: rosgpt.py 안에 Flask 앱이 있고, /rosgpt POST 요청을 처리하는 핸들러가 ROSGPTProxy.post().

클라이언트 측: rosgpt_client_node.py에서 requests.post(...)로 Flask 서버에 요청을 보냄.

즉, 서로 다른 코드인데, 의도적으로 연결된 거야:

[rosgpt_client_node.py] ---HTTP POST---> [rosgpt.py:ROSGPTProxy.post()]

"""