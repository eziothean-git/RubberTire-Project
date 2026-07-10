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
    public float normalSupportSlop = 0.005f;
    public bool enableNormalGroundReactionForces = false;

    // Solver policy, deliberately fixed rather than exposed as tyre identity parameters.
    // Recovery is limited in velocity space, so a light wheel cannot receive the same
    // huge force as a heavy wheel and leave the patch faster than the requested speed.
    private const float NormalRecoveryTime = 0.10f;
    private const float NormalMaxRecoverySpeed = 1.25f;

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

        float dt = Mathf.Max(1e-5f, Time.fixedDeltaTime);
        float penetration = Mathf.Max(0f, sample.pen - Mathf.Max(0f, normalSupportSlop));

        // A pneumatic tyre resists motion in both directions while compressed.
        // Rebound damping subtracts support instead of allowing the spring to launch
        // the wheel with no opposing term.
        float physicalForce = Mathf.Max(0f, springK) * penetration
                            - Mathf.Max(0f, damperC) * relativeNormalVelocity;
        physicalForce = Mathf.Clamp(physicalForce, 0f, Mathf.Max(0f, maxNormalForce));
        float physicalImpulse = physicalForce * dt;

        float stopCompressionImpulse = 0f;
        if (enableStableNormalSupport)
        {
            float effectiveMass = GetEffectiveNormalMass(
                sample.p,
                contactNormal,
                enableNormalGroundReactionForces ? groundBody : null);
            float recoverySpeed = penetration
                                * Mathf.Clamp01(normalSupportERP)
                                / NormalRecoveryTime;
            recoverySpeed = Mathf.Min(recoverySpeed, NormalMaxRecoverySpeed);

            // Stopping compression is dissipative. It can raise an under-tuned spring
            // up to the impulse needed to cancel a fraction of incoming velocity.
            stopCompressionImpulse = effectiveMass
                * Mathf.Max(0f, -relativeNormalVelocity)
                * Mathf.Clamp01(normalSupportVelDamping);
            // The velocity-space guard applies only to the solver's compression
            // stop. It must not clamp the tuned spring branch: the wheel's isolated
            // effective mass does not include the chassis load transmitted through
            // Besiege joints, so using it as a cap made stiffness saturate at roughly
            // 125 N for a 1 kg wheel at 100 Hz regardless of springK.
            float maximumSafeImpulse = effectiveMass
                * Mathf.Max(0f, recoverySpeed - relativeNormalVelocity);
            stopCompressionImpulse = Mathf.Min(stopCompressionImpulse, maximumSafeImpulse);
        }

        // A3: the contact gate fades energy-injecting spring support on flickering
        // contacts. The compression stop is purely dissipative, so it bypasses the
        // gate; gating it under-damped hard landings during fade-in and the
        // deepened spring launched the wheel afterwards.
        float gatedImpulse = physicalImpulse * Mathf.Clamp01(gate);
        if (stopCompressionImpulse > gatedImpulse) gatedImpulse = stopCompressionImpulse;
        float appliedImpulse = gatedImpulse * Mathf.Max(0f, sampleWeight);
        if (appliedImpulse <= 1e-8f) return 0f;

        Vector3 impulse = appliedImpulse * contactNormal;
        Rigidbody.AddForceAtPosition(impulse, sample.p, ForceMode.Impulse);
        if (enableNormalGroundReactionForces && groundBody != null)
            groundBody.AddForceAtPosition(-impulse, sample.p, ForceMode.Impulse);
        return appliedImpulse / dt;
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
