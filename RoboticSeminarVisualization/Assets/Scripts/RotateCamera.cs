using UnityEngine;

public class RotateCamera : MonoBehaviour
{
    [SerializeField] private Camera cam;
    [SerializeField] private Transform center;
    [SerializeField] private float speed = 1;
    [SerializeField] private float distance = -10;
    [SerializeField] private MouseEnteredCheck check;
    
    private Vector3 previousPos;
    
    // Update is called once per frame
    private void Update()
    {
        if(!check.mouseEntered)
            return;
        
        if (Input.GetMouseButtonDown(0))
        {
            previousPos = cam.ScreenToViewportPoint(Input.mousePosition);
        }
        
        if (Input.GetMouseButton(0))
        {
            var newPos = cam.ScreenToViewportPoint(Input.mousePosition);
            var dir = previousPos - newPos;

            cam.transform.position = center.position;
            
            cam.transform.Rotate(Vector3.right, 180 * dir.y * speed);
            cam.transform.Rotate(Vector3.up, -180 * dir.x * speed, Space.World);
            cam.transform.Translate(new Vector3(0,0,distance));
            
            previousPos = newPos;
        }
    }
}
