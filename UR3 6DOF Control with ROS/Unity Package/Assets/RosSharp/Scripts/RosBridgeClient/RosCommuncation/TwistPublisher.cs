/*
© Siemens AG, 2017-2018
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
using RosSharp;
using RosSharp.RosBridgeClient;

namespace RosSharp.RosBridgeClient
{
    public class TwistPublisher : UnityPublisher<MessageTypes.Geometry.Twist>
    {
        private MessageTypes.Geometry.Twist message;
        public Transform tr;
        private int cnt = 0;
    
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
            message = new MessageTypes.Geometry.Twist();
            message.linear = new MessageTypes.Geometry.Vector3();
            message.angular = new MessageTypes.Geometry.Vector3();
        }
        private void UpdateMessage()
        {
            cnt++;
            if (cnt == 100)
            {
                message.linear.x = tr.transform.position.x;
                message.linear.y = tr.transform.position.y;
                message.linear.z = tr.transform.position.z;
                message.angular.x = 0;
                message.angular.y = 0;
                message.angular.z = 0;
                Publish(message);
                cnt = 0;
            }
            
        }
    }
}
