using UnityEngine;
using System.Collections;
using System.Text;
using UnityEngine.UI;
// Needed //////////////////////////////////////////////////
using HoloLensXboxController;
///////////////////////////////////////////////////////////

public class ControllerViconAdjustment : MonoBehaviour {
    
    // Needed //////////////////////////////////////////////////
    private ControllerInput controllerInput;
    ///////////////////////////////////////////////////////////

    // Use this for initialization
    void Start () {
        // Needed //////////////////////////////////////////////////
        controllerInput = new ControllerInput(0, 0.19f);
        // First parameter is the number, starting at zero, of the controller you want to follow.
        // Second parameter is the default “dead” value; meaning all stick readings less than this value will be set to 0.0.
        ///////////////////////////////////////////////////////////
    }

    void Update () {
        // Needed //////////////////////////////////////////////////
        controllerInput.Update();
        ///////////////////////////////////////////////////////////

        if (controllerInput.GetButtonDown(ControllerButton.DPadLeft)) {
            this.gameObject.GetComponent<ViconTracking>().adjustVec.x += .02f;
        }
        if (controllerInput.GetButtonDown(ControllerButton.DPadRight)) {
            this.gameObject.GetComponent<ViconTracking>().adjustVec.x -= .02f;
        }
        if (controllerInput.GetButtonDown(ControllerButton.DPadUp)) {
            this.gameObject.GetComponent<ViconTracking>().adjustVec.z += .02f;
        }
        if (controllerInput.GetButtonDown(ControllerButton.DPadDown)) {
            this.gameObject.GetComponent<ViconTracking>().adjustVec.z -= .02f;
        }
        if (controllerInput.GetButtonDown(ControllerButton.RightShoulder)) {
            this.gameObject.GetComponent<ViconTracking>().adjustVec.y += .02f;
        }
        if (controllerInput.GetButtonDown(ControllerButton.LeftShoulder)) {
            this.gameObject.GetComponent<ViconTracking>().adjustVec.y -= .02f;
        }
    }
}
