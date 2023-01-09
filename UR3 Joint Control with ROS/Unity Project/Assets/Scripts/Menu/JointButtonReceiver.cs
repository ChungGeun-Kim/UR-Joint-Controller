// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License. See LICENSE in the project root for license information.

using UnityEngine;
using UnityEngine.Events;
using UnityEngine.EventSystems;
using Microsoft.MixedReality.Toolkit.UI;
using Microsoft.MixedReality.Toolkit.Input;
using RosSharp;
using RosSharp.RosBridgeClient;
using System.Collections;
using System.Collections.Generic;

public class JointButtonReceiver : ReceiverBase
{
    public override bool HideUnityEvents => true;
    private State lastState;
    public int cnt = 0;
    public bool hasDown;

    // Joint Position을 Publish하는 JointStatePublisher를 참조
    public JointStatePublisher JointStatePublisher;
    
    public JointButtonReceiver(UnityEvent ev): base(ev, "CustomEvent") 
    { 
    }

    public override void OnUpdate(InteractableStates state, Interactable source)
    {
        if (state.CurrentState() != lastState)
        {
            lastState = state.CurrentState();
        }

        // Joint +,- 버튼이 속한 GameObject인 ImageTartget
        GameObject imagetarget = GameObject.Find("ImageTarget");

        // Joint Position을 Publish하는 JointStatePublisher를 참조
        JointStatePublisher = GameObject.FindObjectOfType<JointStatePublisher>();
        
        // 버튼이 눌려져있는 상태인지 아닌지 판단
        if (state.GetState(InteractableStates.InteractableStateEnum.Pressed).Value > 0 && !hasDown)
        {
            hasDown = true;
        }
        else if (state.GetState(InteractableStates.InteractableStateEnum.Pressed).Value < 1)
        {
            hasDown = false;
        }

        // 버튼이 눌려져있는 상태에서만 각 Joint의 Position 값을 변경
        // 각 Joint에 제한을 두어 일정 각을 넘지 못하게 설정
        if (hasDown)
        {
            switch (source.transform.name)
            {
                case "J1pButton":
                    cnt++;
                    if (JointStatePublisher.message.position[0] <= -3.14)
                        JointStatePublisher.message.position[0] = -3.14;
                    if (JointStatePublisher.message.position[0] >= 3.14)
                        JointStatePublisher.message.position[0] = 3.14;
                    if (cnt == 5)
                    {
                        JointStatePublisher.message.position[0] += 0.01;
                        cnt = 0;
                    }
                    break;
                case "J1mButton":
                    cnt++;
                    if (JointStatePublisher.message.position[0] <= -3.14)
                        JointStatePublisher.message.position[0] = -3.14;
                    if (JointStatePublisher.message.position[0] >= 3.14)
                        JointStatePublisher.message.position[0] = 3.14;
                    if (cnt == 5)
                    {
                        JointStatePublisher.message.position[0] -= 0.01;
                        cnt = 0;
                    }
                    break;
                case "J2pButton":
                    cnt++;
                    if (JointStatePublisher.message.position[1] <= -3.14)
                        JointStatePublisher.message.position[1] = -3.14;
                    if (JointStatePublisher.message.position[1] >= 3.14)
                        JointStatePublisher.message.position[1] = 3.14;
                    if (cnt == 5)
                    {
                        JointStatePublisher.message.position[1] += 0.01;
                        cnt = 0;
                    }
                    break;
                case "J2mButton":
                    cnt++;
                    if (JointStatePublisher.message.position[1] <= -3.14)
                        JointStatePublisher.message.position[1] = -3.14;
                    if (JointStatePublisher.message.position[1] >= 3.14)
                        JointStatePublisher.message.position[1] = 3.14;
                    if (cnt == 5)
                    {
                        JointStatePublisher.message.position[1] -= 0.01;
                        cnt = 0;
                    }
                    break;
                case "J3pButton":
                    cnt++;
                    if (JointStatePublisher.message.position[2] <= -3.14)
                        JointStatePublisher.message.position[2] = -3.14;
                    if (JointStatePublisher.message.position[2] >= 3.14)
                        JointStatePublisher.message.position[2] = 3.14;
                    if (cnt == 5)
                    {
                        JointStatePublisher.message.position[2] += 0.01;
                        cnt = 0;
                    }
                    break;
                case "J3mButton":
                    cnt++;
                    if (JointStatePublisher.message.position[2] <= -3.14)
                        JointStatePublisher.message.position[2] = -3.14;
                    if (JointStatePublisher.message.position[2] >= 3.14)
                        JointStatePublisher.message.position[2] = 3.14;
                    if (cnt == 5)
                    {
                        JointStatePublisher.message.position[2] -= 0.01;
                        cnt = 0;
                    }
                    break;
                case "J4pButton":
                    cnt++;
                    if (JointStatePublisher.message.position[3] <= -3.14)
                        JointStatePublisher.message.position[3] = 3.14;
                    if (JointStatePublisher.message.position[3] >= 3.14)
                        JointStatePublisher.message.position[3] -= 3.14;
                    if (cnt == 5)
                    {
                        JointStatePublisher.message.position[3] += 0.01;
                        cnt = 0;
                    }
                    break;
                case "J4mButton":
                    cnt++;
                    if (JointStatePublisher.message.position[3] <= -3.14)
                        JointStatePublisher.message.position[3] = -3.14;
                    if (JointStatePublisher.message.position[3] >= 3.14)
                        JointStatePublisher.message.position[3] = 3.14;
                    if (cnt == 5)
                    {
                        JointStatePublisher.message.position[3] -= 0.01;
                        cnt = 0;
                    }
                    break;
                case "J5pButton":
                    cnt++;
                    if (JointStatePublisher.message.position[4] <= -3.14)
                        JointStatePublisher.message.position[4] = -3.14;
                    if (JointStatePublisher.message.position[4] >= 3.14)
                        JointStatePublisher.message.position[4] = 3.14;
                    if (cnt == 5)
                    {
                        JointStatePublisher.message.position[4] += 0.01;
                        cnt = 0;
                    }
                    break;
                case "J5mButton":
                    cnt++;
                    if (JointStatePublisher.message.position[4] <= -3.14)
                        JointStatePublisher.message.position[4] = -3.14;
                    if (JointStatePublisher.message.position[4] >= 3.14)
                        JointStatePublisher.message.position[4] = 3.14;
                    if (cnt == 5)
                    {
                        JointStatePublisher.message.position[4] -= 0.01;
                        cnt = 0;
                    }
                    break;
                case "J6pButton":
                    cnt++;
                    if (JointStatePublisher.message.position[5] <= -3.14)
                        JointStatePublisher.message.position[5] = -3.14;
                    if (JointStatePublisher.message.position[5] >= 3.14)
                        JointStatePublisher.message.position[5] = 3.14;
                    if (cnt == 5)
                    {
                        JointStatePublisher.message.position[5] += 0.01;
                        cnt = 0;
                    }
                    break;
                case "J6mButton":
                    cnt++;
                    if (JointStatePublisher.message.position[5] <= -3.14)
                        JointStatePublisher.message.position[5] = -3.14;
                    if (JointStatePublisher.message.position[5] >= 3.14)
                        JointStatePublisher.message.position[5] = 3.14;
                    if (cnt == 5)
                    {
                        JointStatePublisher.message.position[5] -= 0.01;
                        cnt = 0;
                    }
                    break;
                default:
                    break;
            }
            
        }

        
    }
}

