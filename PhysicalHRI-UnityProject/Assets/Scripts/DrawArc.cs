using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DrawArc : MonoBehaviour {

    public int numberOfPoints = 30;
    public float ringRadius = 1;
    public int arcDegrees = 250;

    void Update() {
        LineRenderer linerenderer = this.GetComponent<LineRenderer>();
        linerenderer.positionCount = numberOfPoints;

        for (int i = 0; i < numberOfPoints; i++) {
            float angle = (i * ((Mathf.PI / 180) * arcDegrees) / numberOfPoints) - this.transform.eulerAngles.y / 32;
            Vector3 pos = new Vector3(Mathf.Cos(angle), this.transform.localPosition.y, Mathf.Sin(angle)) * ringRadius;
            pos = new Vector3(pos.x + this.transform.position.x, pos.y * 2, pos.z + this.transform.position.z);
            linerenderer.SetPosition(i, pos);
        }
    }
}
