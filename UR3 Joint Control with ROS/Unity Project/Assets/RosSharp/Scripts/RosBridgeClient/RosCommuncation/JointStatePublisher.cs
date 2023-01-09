/*
© Siemens AG, 2017-2019
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
using UnityEngine;
using UnityEngine.Events;
using Microsoft.MixedReality.Toolkit.UI;
using Microsoft.MixedReality.Toolkit.Input;
using RosSharp;
using RosSharp.RosBridgeClient;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using System.Threading;

namespace RosSharp.RosBridgeClient
{
    public class JointStatePublisher : UnityPublisher<MessageTypes.Sensor.JointState>
    {
        public List<JointStateReader> JointStateReaders;
        public string FrameId = "Unity";
        public MessageTypes.Sensor.JointState temp;
        public MessageTypes.Sensor.JointState message;
        public int cnt = 0;

        protected override void Start()
        {
            base.Start();
            InitializeMessage();
        }

        private void FixedUpdate()
        {
            UpdateMessage();
        }

        private void InitializeMessage()
        {
            int jointStateLength = JointStateReaders.Count;

            // Publish할 message
            message = new MessageTypes.Sensor.JointState
            {
                header = new MessageTypes.Std.Header { frame_id = FrameId },
                name = new string[jointStateLength],
                position = new double[jointStateLength],
                velocity = new double[jointStateLength],
                effort = new double[jointStateLength]
            };

            // Joint 값이 변하지 않을 때 Publish를 제한하기 위한 temp
            temp = new MessageTypes.Sensor.JointState
            {
                header = new MessageTypes.Std.Header { frame_id = FrameId },
                name = new string[jointStateLength],
                position = new double[jointStateLength],
                velocity = new double[jointStateLength],
                effort = new double[jointStateLength]
            };

            message.header.Update();
            temp.header.Update();

            for (int i = 0; i < JointStateReaders.Count; i++)
                UpdateJointState(i);
        }

        private void UpdateMessage()
        {
            // 시작 자세 설정
            if (cnt == 0)
            {
                message.position[0] = 3.141592;
                message.position[1] = -1.570796;
                message.position[2] = 0;
                message.position[3] = -1.570796;
                message.position[4] = -3.141592;
                message.position[5] = 0;
                Publish(message);
                cnt++;
            }
            
            // Base
            if (message.position[0] > temp.position[0])
            {  
                Publish(message);
                temp.position[0] = message.position[0]; 
            }
            if (message.position[0] < temp.position[0])
            {
                Publish(message);
                temp.position[0] = message.position[0]; 
            }

            // Shoulder
            if (message.position[1] > temp.position[1])
            {
                Publish(message);
                temp.position[1] = message.position[1];
            }
            if (message.position[1] < temp.position[1])
            {
                Publish(message);
                temp.position[1] = message.position[1];
            }

            // Elbow
            if (message.position[2] > temp.position[2])
            {
                Publish(message);
                temp.position[2] = message.position[2];
            }
            if (message.position[2] < temp.position[2])
            {
                Publish(message);
                temp.position[2] = message.position[2]; 
            }

            // Wrist 1
            if (message.position[3] > temp.position[3])
            {
                Publish(message);
                temp.position[3] = message.position[3];  
            }
            if (message.position[3] < temp.position[3])
            {
                Publish(message);
                temp.position[3] = message.position[3];  
            }

            // Wrist 2
            if (message.position[4] > temp.position[4])
            {
                Publish(message);
                temp.position[4] = message.position[4];
            }
            if (message.position[4] < temp.position[4])
            {
                Publish(message);
                temp.position[4] = message.position[4];
            }

            // Wrist 3
            if (message.position[5] > temp.position[5])
            {
                Publish(message);
                temp.position[5] = message.position[5];
            }
            if (message.position[5] < temp.position[5])
            {
                Publish(message);
                temp.position[5] = message.position[5];
            }
        }

        private void UpdateJointState(int i)
        {

            JointStateReaders[i].Read(
                out message.name[i],
                out float position,
                out float velocity,
                out float effort);

            message.position[i] = position;
            message.velocity[i] = velocity;
            message.effort[i] = effort;

            temp.position[i] = position;
            temp.velocity[i] = velocity;
            temp.effort[i] = effort;
        }


    }
}
