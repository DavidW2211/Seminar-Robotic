using UnityEngine;
using UnityEngine.EventSystems;

public class MouseEnteredCheck : MonoBehaviour, IPointerEnterHandler, IPointerExitHandler
{
    public bool mouseEntered = false;
        
    public void OnPointerEnter(PointerEventData eventData)
    {
        mouseEntered = true;
    }

    public void OnPointerExit(PointerEventData eventData)
    {
        mouseEntered = false;
    }
}