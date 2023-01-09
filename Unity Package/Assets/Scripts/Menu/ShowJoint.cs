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
    public TMP_Text Base, Shoulder, Elbow;
    public PoseStampedPublisher PoseStampedPublisher;
    public float X, Y, Z;
    void Start()
    {
        PoseStampedPublisher = GameObject.FindObjectOfType<PoseStampedPublisher>();
    }

    void Update()
    {
        X = Mathf.Round(PoseStampedPublisher.rot.x);
        Y = Mathf.Round(PoseStampedPublisher.rot.y);
        Z = Mathf.Round(PoseStampedPublisher.rot.z);
        
        Base.text = X.ToString();
        Shoulder.text = Y.ToString();
        Elbow.text = Z.ToString();
    }
    
}
