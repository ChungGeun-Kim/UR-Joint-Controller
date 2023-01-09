// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License. See LICENSE in the project root for license information.

using UnityEngine;
using UnityEngine.UI;
using RosSharp.RosBridgeClient;
using System;
using System.Collections;
using System.Collections.Generic;
using TMPro;

public class ShowJoint : MonoBehaviour
{
    // Unity의 TextMeshPro Text에 접근하기 위해 TMP_Text 자료형 사용
    // TextMeshPro는 3D 공간에 텍스트를 띄워주는 패키지
    public TMP_Text Base, Shoulder, Elbow, Wrist_1, Wrist_2, Wrist_3;
    
    // 각 Joint 값에 접근하기위해 Joint 값이 저장되어있는 클래스 참조 
    private JointStatePublisher JointStatePublisher;
    
    void Start()
    {
    }

    void Update()
    {
        JointStatePublisher = GameObject.FindObjectOfType<JointStatePublisher>();

        var Base_joint = Math.Round(JointStatePublisher.message.position[0], 3);
        var Shoulder_joint = Math.Round(JointStatePublisher.message.position[1], 3);
        var Elbow_joint = Math.Round(JointStatePublisher.message.position[2], 3);
        var Wrist1_joint = Math.Round(JointStatePublisher.message.position[3], 3);
        var Wrist2_joint = Math.Round(JointStatePublisher.message.position[4], 3);
        var Wrist3_joint = Math.Round(JointStatePublisher.message.position[5], 3);

        Base.text = Base_joint.ToString();
        Shoulder.text = Shoulder_joint.ToString();
        Elbow.text = Elbow_joint.ToString();
        Wrist_1.text = Wrist1_joint.ToString();
        Wrist_2.text = Wrist2_joint.ToString();
        Wrist_3.text = Wrist3_joint.ToString();
    }
    
}
