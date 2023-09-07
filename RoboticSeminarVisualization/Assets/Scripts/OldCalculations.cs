using System;
using TMPro;
using UnityEngine;

public class OldCalculations : MonoBehaviour
{
    [SerializeField] private Transform sensorBase;
    [SerializeField] private Transform sensorSkin;
    [SerializeField] private Transform rotationDiff;
    [SerializeField] private Transform stage3;
    [SerializeField] private TMP_Text txt;
    [SerializeField] private TMP_Text txt2;
    private Quaternion offset;
    
    private void Start()
    {
        var qBase = sensorBase.rotation;
        var qSkin = sensorSkin.rotation;
        offset = Quaternion.Inverse(qSkin) * qBase;   //Important! Not the other way around!
    }
    
    // Update is called once per frame
    void Update()
    {
        var qBase = sensorBase.rotation;
        var qSkin = sensorSkin.rotation;
        var qDiff = Quaternion.Inverse(offset) * (Quaternion.Inverse(qSkin) * qBase);   //Important! Not the other way around!
        
        rotationDiff.rotation = qDiff;
        var newDiff = qDiff * Vector3.up;
        
        #region OldVariants
        /* Version 1:
        var x = newDiff.x;
        var y = Mathf.Sqrt(1 - (x * x));

        if (newDiff.y < 0)
            y *= -1;
        
        var angle = 90 - Mathf.Atan2(y, x) * (180 / Math.PI);

        if (angle > 180)
        {
            angle = angle - 360;
        }*/

        /* Version 2:
        var x = newDiff.x;
        var y = newDiff.y;
        var angleZAxis = 90 - Mathf.Acos(x) * (180 / Math.PI);

        angleZAxis = x switch
        {
            < 0 when y < 0 => -180 - angleZAxis, //Third Quadrant
            >= 0 when y < 0 => 180 - angleZAxis, //Fourth Quadrant
            _ => angleZAxis                      //First & Second Quadrant
        };

        var z = newDiff.z;
        var angleXAxis = 90 - Mathf.Acos(z) * (180 / Math.PI);
        angleXAxis = z switch
        {
            < 0 when y < 0 => -180 - angleXAxis, //Third Quadrant
            >= 0 when y < 0 => 180 - angleXAxis, //Fourth Quadrant
            _ => angleXAxis                      //First & Second Quadrant
        };
        */
        #endregion
        
        var x = newDiff.x;
        var y = newDiff.y;
        var z = newDiff.z;
        
        var angleZAxis = Vector3.Angle(new Vector3(x, y, 0), Vector3.up);

        if (x < 0)
            angleZAxis *= -1;

        newDiff = Quaternion.AngleAxis(angleZAxis, Vector3.forward) * newDiff;
        
        var angleXAxis = Vector3.Angle(new Vector3(0, newDiff.y, newDiff.z), Vector3.up);
        
        if (z < 0)
            angleXAxis *= -1;
        
        
        stage3.rotation = rotationDiff.rotation;
        stage3.Rotate(new Vector3(-angleXAxis,0, angleZAxis), Space.World);
        
        txt.text = "Z-Axis Error: " + angleZAxis.ToString("0.00");
        txt2.text = "X-Axis Error: " + angleXAxis.ToString("0.00");;
    }
}
