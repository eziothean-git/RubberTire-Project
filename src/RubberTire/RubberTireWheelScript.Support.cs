using UnityEngine;
using Modding;

public partial class RubberTireWheelScript
{
    // Normal/support model parameters.
    public float springK = 3000f;
    public float damperC = 25f;
    public float maxNormalForce = 200000f;
    public bool enableStableNormalSupport = true;
    public float normalSupportERP = 0.35f;
    public float normalSupportVelDamping = 1.0f;
    public float normalSupportMassScale = 4.0f;
    public float normalSupportSlop = 0.005f;
    public bool enableNormalGroundReactionForces = false;

    private MSlider uiK, uiC;
    private MToggle uiStableNormal, uiNormalGroundReaction;
    private MSlider uiNormalERP, uiNormalVelDamping, uiNormalMassScale, uiNormalSlop;

    private void CreateSupportMapperControls()
    {
        uiK = AddSlider("K (Spring)", "k", springK, 0f, 200000f);
        uiC = AddSlider("C (Damper)", "c", damperC, 0f, 5000f);
        uiStableNormal = AddToggle("Stable Normal Support", "nStable", enableStableNormalSupport);
        uiNormalERP = AddSlider("Normal ERP", "nErp", normalSupportERP, 0f, 1f);
        uiNormalVelDamping = AddSlider("Normal Vel Damping", "nDamp", normalSupportVelDamping, 0f, 2f);
        uiNormalMassScale = AddSlider("Normal Mass Scale", "nMass", normalSupportMassScale, 0.25f, 20f);
        uiNormalSlop = AddSlider("Normal Slop", "nSlop", normalSupportSlop, 0f, 0.05f);
    }

    private void CreateSupportReactionMapperControl()
    {
        uiNormalGroundReaction = AddToggle("ADV: Normal Reaction", "advNReact", enableNormalGroundReactionForces);
    }

    private void SyncSupportParamsFromUI()
    {
        if (uiK != null) springK = uiK.Value;
        if (uiC != null) damperC = uiC.Value;
        if (uiStableNormal != null) enableStableNormalSupport = uiStableNormal.IsActive;
        if (uiNormalERP != null) normalSupportERP = uiNormalERP.Value;
        if (uiNormalVelDamping != null) normalSupportVelDamping = uiNormalVelDamping.Value;
        if (uiNormalMassScale != null) normalSupportMassScale = uiNormalMassScale.Value;
        if (uiNormalSlop != null) normalSupportSlop = uiNormalSlop.Value;
        if (uiNormalGroundReaction != null) enableNormalGroundReactionForces = uiNormalGroundReaction.IsActive;
    }

    private float EvaluateAndApplySupportForce(
        ContactSample sample,
        Vector3 contactNormal,
        Rigidbody groundBody,
        float gate,
        float sampleWeight)
    {
        Vector3 wheelVelocity = Rigidbody.GetPointVelocity(sample.p);
        Vector3 groundVelocity = groundBody != null
            ? groundBody.GetPointVelocity(sample.p)
            : Vector3.zero;
        float relativeNormalVelocity = Vector3.Dot(
            wheelVelocity - groundVelocity,
            contactNormal);

        float rawForce = springK * sample.pen;
        if (damperC > 0f)
        {
            float compressionSpeed = -relativeNormalVelocity;
            if (compressionSpeed > 0f) rawForce += damperC * compressionSpeed;
        }

        float stableForce = BuildStableNormalSupportForce(
            sample.pen,
            relativeNormalVelocity,
            sample.p,
            contactNormal,
            groundBody);
        if (stableForce > rawForce) rawForce = stableForce;
        rawForce = Mathf.Clamp(rawForce, 0f, maxNormalForce);

        float normalLoad = rawForce * gate * sampleWeight;
        if (normalLoad <= 1e-6f) return 0f;

        Vector3 force = normalLoad * contactNormal;
        Rigidbody.AddForceAtPosition(force, sample.p, ForceMode.Force);
        if (enableNormalGroundReactionForces && groundBody != null)
            groundBody.AddForceAtPosition(-force, sample.p, ForceMode.Force);
        return normalLoad;
    }

    private float BuildStableNormalSupportForce(
        float penetration,
        float relativeNormalVelocity,
        Vector3 point,
        Vector3 normal,
        Rigidbody groundRb)
    {
        if (!enableStableNormalSupport) return 0f;
        if (!HasRigidbody) return 0f;
        if (normal.sqrMagnitude < 1e-10f) return 0f;

        float dt = Mathf.Max(1e-5f, Time.fixedDeltaTime);
        float pen = Mathf.Max(0f, penetration - Mathf.Max(0f, normalSupportSlop));

        float targetVN = pen * Mathf.Clamp01(normalSupportERP) / dt;
        float dv = targetVN - relativeNormalVelocity * Mathf.Max(0f, normalSupportVelDamping);
        if (dv <= 1e-5f) return 0f;

        float effectiveMass = GetEffectiveNormalMass(point, normal, groundRb);
        effectiveMass *= Mathf.Max(0.01f, normalSupportMassScale);
        return effectiveMass * dv / dt;
    }

    private float GetEffectiveNormalMass(Vector3 point, Vector3 normal, Rigidbody groundRb)
    {
        float invMass = GetPointInverseMassAlongAxis(Rigidbody, point, normal);
        if (groundRb != null) invMass += GetPointInverseMassAlongAxis(groundRb, point, normal);

        if (invMass > 1e-6f) return 1f / invMass;
        if (HasRigidbody && Rigidbody.mass > 1e-6f) return Rigidbody.mass;
        return 1f;
    }

    private float GetPointInverseMassAlongAxis(Rigidbody rb, Vector3 point, Vector3 axisWorld)
    {
        if (rb == null) return 0f;
        if (rb.isKinematic) return 0f;
        if (axisWorld.sqrMagnitude < 1e-10f) return 0f;

        axisWorld.Normalize();
        float invMass = rb.mass > 1e-6f ? 1f / rb.mass : 0f;

        Vector3 r = point - rb.worldCenterOfMass;
        Vector3 rn = Vector3.Cross(r, axisWorld);
        Vector3 invIrn = MultiplyWorldInverseInertia(rb, rn);
        float angular = Vector3.Dot(Vector3.Cross(invIrn, r), axisWorld);
        if (angular < 0f) angular = 0f;
        return invMass + angular;
    }

    private Vector3 MultiplyWorldInverseInertia(Rigidbody rb, Vector3 v)
    {
        if (rb == null) return Vector3.zero;

        Quaternion principalToWorld = rb.rotation * rb.inertiaTensorRotation;
        Vector3 vp = Quaternion.Inverse(principalToWorld) * v;
        Vector3 inertia = rb.inertiaTensor;

        vp.x = inertia.x > 1e-6f ? vp.x / inertia.x : 0f;
        vp.y = inertia.y > 1e-6f ? vp.y / inertia.y : 0f;
        vp.z = inertia.z > 1e-6f ? vp.z / inertia.z : 0f;
        return principalToWorld * vp;
    }
}
