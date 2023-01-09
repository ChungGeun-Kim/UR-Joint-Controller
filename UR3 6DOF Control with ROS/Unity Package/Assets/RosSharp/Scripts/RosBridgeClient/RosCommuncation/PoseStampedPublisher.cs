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
using UnityEngine.UI;
using UnityEngine.Events;
using Microsoft.MixedReality.Toolkit.UI;
using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.Physics;
using Microsoft.MixedReality.Toolkit.Utilities;
using RosSharp;
using RosSharp.RosBridgeClient;
using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using System.Threading;

namespace RosSharp.RosBridgeClient
{
    public class PoseStampedPublisher : UnityPublisher<MessageTypes.Geometry.PoseStamped>
    {
        public Transform PublishedTransform;
        public string FrameId = "Unity";
        public MessageTypes.Geometry.PoseStamped message;
        public PinchSlider slider_1, slider_2, slider_3;
        private ObjectManipulator obj;
        public Vector3 pointer, rot, euler;
        public Quaternion ori;
        private bool start;
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
            message = new MessageTypes.Geometry.PoseStamped
            {
                header = new MessageTypes.Std.Header()
                {
                    frame_id = FrameId
                }
            };
        }

        private void UpdateMessage()
        {
            obj = GameObject.FindObjectOfType<ObjectManipulator>();
            start = obj.isManipulationStarted;
            pointer = obj.GetPointersGrabPoint();

            if (slider_1.SliderValue == 0.0)
                rot.x = 0.1f;
            else
                rot.x = slider_1.SliderValue * 359.9f;

            if (slider_2.SliderValue == 0.0)
                rot.y = 0.1f;
            else
                rot.y = slider_2.SliderValue * 359.9f;

            if (slider_3.SliderValue == 0.0)
                rot.z = 0.1f;
            else
                rot.z = slider_3.SliderValue * 359.9f;

            ori = Quaternion.Euler(rot);

            message.header.Update();

            message.pose.position.x = pointer.x;
            message.pose.position.y = pointer.y;
            message.pose.position.z = pointer.z;

            message.pose.orientation.x = ori.x;
            message.pose.orientation.y = ori.y;
            message.pose.orientation.z = ori.z;
            message.pose.orientation.w = ori.w;

            cnt++;
            if (cnt == 10)
            {
                Publish(message);
                cnt = 1;
            }
            /*
            if (start || slider_1.status || slider_2.status || slider_3.status)
            {
                
            }
            */
        }

        private MessageTypes.Geometry.Point GetGeometryPoint(Vector3 position)
        {
            MessageTypes.Geometry.Point geometryPoint = new MessageTypes.Geometry.Point();
            geometryPoint.x = position.x;
            geometryPoint.y = position.y;
            geometryPoint.z = position.z;
            return geometryPoint;
        }

        private MessageTypes.Geometry.Quaternion GetGeometryQuaternion(Quaternion quaternion)
        {
            MessageTypes.Geometry.Quaternion geometryQuaternion = new MessageTypes.Geometry.Quaternion();
            geometryQuaternion.x = quaternion.x;
            geometryQuaternion.y = quaternion.y;
            geometryQuaternion.z = quaternion.z;
            geometryQuaternion.w = quaternion.w;
            return geometryQuaternion;
        }
    }
}
