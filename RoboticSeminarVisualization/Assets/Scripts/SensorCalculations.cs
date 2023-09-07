using TMPro;
using UnityEngine;
using UnityEngine.UI;
using Random = UnityEngine.Random;

public class SensorCalculations : MonoBehaviour
{
    [SerializeField] private Transform sensorBase;
    [SerializeField] private Transform sensorSkin;
    [SerializeField] private Transform rotationDiff;
    [SerializeField] private Transform stage2;
    [SerializeField] private Transform stage3;
    [SerializeField] private TMP_Text txt;
    [SerializeField] private TMP_Text txt2;
    [SerializeField] private bool useAutomaticFix = false;
    [SerializeField] private bool useAutomaticFix2Axis = false;
    [SerializeField] private float speed = 10;
    [SerializeField] private bool useAutoArmMovement = false;
    [SerializeField] private Transform arm;
    [SerializeField] private Vector2 xAngleRange;
    [SerializeField] private Vector2 yAngleRange;
    [SerializeField] private Vector2 zAngleRange;
    [SerializeField] private float armSpeed;
    [SerializeField] private Slider xSlider;
    [SerializeField] private Slider ySlider;
    [SerializeField] private Slider zSlider;
    [SerializeField] private GameObject casing;
    [SerializeField] private GameObject topping;
    
    private Quaternion offset;
    private float minError;
    
    //Arm
    private Quaternion armTargetRot = Quaternion.identity;
    //Sliders
    private float currXRot;
    private float currYRot;
    private float currZRot;
    
    private void Start()
    {
        var qBase = sensorBase.rotation;
        var qSkin = sensorSkin.rotation;
        offset = Quaternion.Inverse(qSkin) * qBase;   //Important! Not the other way around!
    }

    private void Update()
    {
        if(Input.GetKeyDown(KeyCode.Escape))
            Application.Quit();

        var qBase = sensorBase.rotation;
        var qSkin = sensorSkin.rotation;
        var qDiff = Quaternion.Inverse(offset) * (Quaternion.Inverse(qSkin) * qBase);   //Important! Not the other way around!
        
        rotationDiff.rotation = qDiff;
        var newDiff = qDiff * Vector3.up;

        /* Correct Version:*/
        var x = newDiff.x;
        var y = newDiff.y;

        var angleZAxis = Vector3.Angle(new Vector3(x, y, 0), Vector3.up);

        if (x < 0)
            angleZAxis *= -1;

        var z = newDiff.z;
        var angleXAxis = Vector3.Angle(new Vector3(0, y, z), Vector3.up);
        
        if (z < 0)
            angleXAxis *= -1;


        #region ArmMovement
        if (useAutoArmMovement)
        {
            arm.rotation = Quaternion.RotateTowards(arm.rotation, armTargetRot, armSpeed * Time.deltaTime);

            if (armTargetRot == arm.rotation)
            {
                var xRot = Random.Range(xAngleRange.x, xAngleRange.y);
                var yRot = Random.Range(yAngleRange.x, yAngleRange.y);
                var zRot = Random.Range(zAngleRange.x, zAngleRange.y);
                armTargetRot = Quaternion.Euler(xRot, yRot, zRot);
            }
        }
        #endregion

        stage2.rotation = Quaternion.identity;
        stage2.Rotate(new Vector3(0,0,(float) -angleZAxis), Space.Self);
        
        stage3.rotation = rotationDiff.rotation;
        stage3.Rotate(new Vector3(0,0,(float) angleZAxis), Space.World);
        

        txt.text = "Z-Axis Error: " + angleZAxis.ToString("0.00");
        txt2.text = "X-Axis Error: " + angleXAxis.ToString("0.00");;
        
        if(!useAutomaticFix)
            return;
        
        if (Mathf.Abs(angleZAxis) > minError)
        {
            switch (angleZAxis)
            {
                case > 0:
                    sensorBase.RotateAround(sensorBase.position, arm.forward,
                        speed * Time.deltaTime);
                    break;
                case < 0:
                    sensorBase.RotateAround(sensorBase.position, arm.forward,
                        -speed * Time.deltaTime);
                    break;
            }
        }
        
        if(!useAutomaticFix2Axis)
            return;

        if (Mathf.Abs(angleXAxis) > minError)
        {
            switch (angleXAxis)
            {
                case > 0:
                    sensorBase.RotateAround(sensorBase.position, arm.right, -speed * Time.deltaTime);
                    break;
                case < 0:
                    sensorBase.RotateAround(sensorBase.position, arm.right, speed * Time.deltaTime);
                    break;
            }   
        }
    }


    public void ToggleVisualCasing(bool value)
    {
        casing.SetActive(value);
    }
    
    public void ToggleVisualTopping(bool value)
    {
        topping.SetActive(value);
    }
    
    public void ToggleAutoArmMovement(bool value)
    {
        useAutoArmMovement = value;
    }

    public void OnChangeSkinDriftFix(int value)
    {
        switch (value)
        {
            case 0:
                useAutomaticFix = false;
                useAutomaticFix2Axis = false;
                break;
            case 1:
                useAutomaticFix = true;
                useAutomaticFix2Axis = false;
                break;
            case 2:
                useAutomaticFix = true;
                useAutomaticFix2Axis = true;
                break;
        }
    }

    public void OnMinErrorChanged(string value)
    {
        minError = float.Parse(value);
    }
    
    public void ResetArmRotation()
    {
        arm.rotation = Quaternion.identity;
    }

    public void OnChangeXRotation(float value)
    {
        currXRot = value;
        UpdateBaseRotation();
    }
    
    public void OnChangeYRotation(float value)
    {
        currYRot = value;
        UpdateBaseRotation();
    }
    
    public void OnChangeZRotation(float value)
    {
        currZRot = value;
        UpdateBaseRotation();
    }

    private void UpdateBaseRotation()
    {
        sensorBase.localRotation = Quaternion.Euler(currXRot, currYRot, currZRot);
    }

    public void ResetBaseRotation()
    {
        currXRot = currYRot = currZRot = 0;
        sensorBase.localRotation = Quaternion.identity;
        xSlider.value = 0;
        ySlider.value = 0;
        zSlider.value = 0;
    }
}
