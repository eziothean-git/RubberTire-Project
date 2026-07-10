using System.Collections.Generic;
using UnityEngine;
using Modding;

public partial class RubberTireWheelScript
{
    public bool enableTireModel = true;
    public float muStatic = 1.40f;
    public float muKinetic = 1.20f;
    public float vStatic = 0.25f;
    public float forceFilterTau = 0.03f;

    public bool enableSinglePassLoadScaling = true;
    public bool enableCombinedSlipFriction = false;
    public float longitudinalGripScale = 1.0f;
    public float lateralGripScale = 1.0f;

    public bool enableModernLowSpeedTire = true;
    public float lowSpeedRelaxSpeedFloor = 1.0f;
    public float lowSpeedCreepSpeed = 0.15f;
    public float lowSpeedCreepBlend = 0.75f;
    public float lowSpeedShearDampingC = 1200f;
    public float axleSpinDampingK = 250f;
    public float axleAirDampingK = 2f;
    public float vEps = 0.2f;

    public bool enableTireRelaxation = true;
    public float relaxLength = 0.8f;
    public float shearK = 50000f;
    public float shearC = 0f;
    public float maxShearDisp = 0.15f;
    public bool resetShearOnNoContact = true;

    public bool decoupleTireForceAndTorque = true;
    public bool enableDecoupledTireForceApplication = false;

    private MSlider uiMuS, uiMuK, uiVStatic, uiFTau;
    private MToggle uiEnableTire, uiRelax, uiDecouple;
    private MSlider uiRelaxL, uiShearK, uiShearC, uiMaxShear;
    private MToggle uiSinglePassLoad, uiCombinedSlip, uiModernLowSpeed, uiDecoupledApply;
    private MSlider uiLongGrip, uiLatGrip, uiLowRelaxFloor, uiLowCreepSpeed;
    private MSlider uiLowCreepBlend, uiLowShearDamping, uiAxleSpinDamping, uiAxleAirDamping;

    private class TirePointState
    {
        public Vector3 shearDispWorld = Vector3.zero;
        public Vector3 FtireFiltered = Vector3.zero;
        public int lastSeenStep;
    }

    private readonly Dictionary<ContactKey, TirePointState> pointStates =
        new Dictionary<ContactKey, TirePointState>(64);
    private readonly List<ContactKey> pointStatesToRemove = new List<ContactKey>(64);

    private void CreateLateralMapperControls()
    {
        uiMuS = AddSlider("Mu Static", "muS", muStatic, 0f, 3f);
        uiMuK = AddSlider("Mu Kinetic", "muK", muKinetic, 0f, 3f);
        uiVStatic = AddSlider("vStatic (m/s)", "vStatic", vStatic, 0.01f, 2.0f);
        uiFTau = AddSlider("Force Filter Tau (s)", "fTau", forceFilterTau, 0f, 0.20f);

        uiSinglePassLoad = AddToggle("ADV: Single Load Scale", "advLoad", enableSinglePassLoadScaling);
        uiCombinedSlip = AddToggle("ADV: Combined Slip", "advSlip", enableCombinedSlipFriction);
        uiLongGrip = AddSlider("ADV: Long Grip Scale", "muLong", longitudinalGripScale, 0f, 3f);
        uiLatGrip = AddSlider("ADV: Lat Grip Scale", "muLat", lateralGripScale, 0f, 3f);
        uiModernLowSpeed = AddToggle("Modern Low Speed Tire", "mls", enableModernLowSpeedTire);
        uiLowRelaxFloor = AddSlider("LowSpeed Relax Floor", "mlsRf", lowSpeedRelaxSpeedFloor, 0.05f, 5.0f);
        uiLowCreepSpeed = AddSlider("LowSpeed Creep Speed", "mlsCv", lowSpeedCreepSpeed, 0.01f, 1.0f);
        uiLowCreepBlend = AddSlider("LowSpeed Creep Blend", "mlsCb", lowSpeedCreepBlend, 0.05f, 3.0f);
        uiLowShearDamping = AddSlider("LowSpeed Shear Damping", "mlsC", lowSpeedShearDampingC, 0f, 10000f);
        uiAxleSpinDamping = AddSlider("Axle Spin Damping", "axDmp", axleSpinDampingK, 0f, 5000f);
        uiAxleAirDamping = AddSlider("Axle Air Damping", "airDmp", axleAirDampingK, 0f, 100f);

        uiEnableTire = AddToggle("Tire Model", "tire", enableTireModel);
        uiRelax = AddToggle("Tire Relaxation", "relax", enableTireRelaxation);
        uiRelaxL = AddSlider("Relax Length", "relaxL", relaxLength, 0.05f, 5.0f);
        uiShearK = AddSlider("Shear K (N/m)", "shK", shearK, 1000f, 200000f);
        uiShearC = AddSlider("Shear C (N*s/m)", "shC", shearC, 0f, 5000f);
        uiMaxShear = AddSlider("Max Shear Disp", "shMax", maxShearDisp, 0.01f, 0.50f);

        uiDecouple = AddToggle("Decouple F/T", "decouple", decoupleTireForceAndTorque);
        uiDecoupledApply = AddToggle("ADV: Decoupled Apply", "advDec", enableDecoupledTireForceApplication);
    }

    private void SyncLateralParamsFromUI()
    {
        if (uiMuS != null) muStatic = uiMuS.Value;
        if (uiMuK != null) muKinetic = uiMuK.Value;
        if (uiVStatic != null) vStatic = uiVStatic.Value;
        if (uiFTau != null) forceFilterTau = uiFTau.Value;

        if (uiSinglePassLoad != null) enableSinglePassLoadScaling = uiSinglePassLoad.IsActive;
        if (uiCombinedSlip != null) enableCombinedSlipFriction = uiCombinedSlip.IsActive;
        if (uiLongGrip != null) longitudinalGripScale = uiLongGrip.Value;
        if (uiLatGrip != null) lateralGripScale = uiLatGrip.Value;
        if (uiModernLowSpeed != null) enableModernLowSpeedTire = uiModernLowSpeed.IsActive;
        if (uiLowRelaxFloor != null) lowSpeedRelaxSpeedFloor = uiLowRelaxFloor.Value;
        if (uiLowCreepSpeed != null) lowSpeedCreepSpeed = uiLowCreepSpeed.Value;
        if (uiLowCreepBlend != null) lowSpeedCreepBlend = uiLowCreepBlend.Value;
        if (uiLowShearDamping != null) lowSpeedShearDampingC = uiLowShearDamping.Value;
        if (uiAxleSpinDamping != null) axleSpinDampingK = uiAxleSpinDamping.Value;
        if (uiAxleAirDamping != null) axleAirDampingK = uiAxleAirDamping.Value;

        if (uiEnableTire != null) enableTireModel = uiEnableTire.IsActive;
        if (uiRelax != null) enableTireRelaxation = uiRelax.IsActive;
        if (uiRelaxL != null) relaxLength = uiRelaxL.Value;
        if (uiShearK != null) shearK = uiShearK.Value;
        if (uiShearC != null) shearC = uiShearC.Value;
        if (uiMaxShear != null) maxShearDisp = uiMaxShear.Value;
        if (uiDecouple != null) decoupleTireForceAndTorque = uiDecouple.IsActive;
        if (uiDecoupledApply != null) enableDecoupledTireForceApplication = uiDecoupledApply.IsActive;
    }

    private Vector3 EvaluateAndApplyTireForce(
        ContactSample sample,
        Vector3 contactNormal,
        Rigidbody groundBody,
        Vector3 wheelAxis,
        float normalLoad,
        float gate,
        float sampleWeight,
        float fixedDeltaTime)
    {
        Vector3 tireForce = Vector3.zero;
        if (!enableTireModel) return tireForce;

        Vector3 groundVelocity = groundBody != null
            ? groundBody.GetPointVelocity(sample.p)
            : Vector3.zero;
        Vector3 wheelVelocity = Rigidbody.GetPointVelocity(sample.p);
        Vector3 relativeVelocity = wheelVelocity - groundVelocity;

        Vector3 forward = ProjectOnPlane(Vector3.Cross(contactNormal, wheelAxis), contactNormal);
        if (forward.sqrMagnitude < 1e-6f)
            forward = ProjectOnPlane(Vector3.Cross(contactNormal, transform.right), contactNormal);
        if (forward.sqrMagnitude <= 1e-6f) return tireForce;
        forward.Normalize();

        Vector3 side = ProjectOnPlane(wheelAxis, contactNormal);
        if (side.sqrMagnitude < 1e-6f)
            side = ProjectOnPlane(Vector3.Cross(forward, contactNormal), contactNormal);
        if (side.sqrMagnitude > 1e-6f)
        {
            side.Normalize();
            if (Vector3.Dot(side, wheelAxis) < 0f) side = -side;
        }
        else
        {
            side = Vector3.zero;
        }

        Vector3 slipVelocity = ProjectOnPlane(relativeVelocity, contactNormal);
        float slipSpeed = slipVelocity.magnitude;

        if (enableTireRelaxation)
        {
            TirePointState state = GetOrCreatePointState(sample.col, sample.p);
            state.lastSeenStep = fixedStepCounter;

            if (enableSinglePassLoadScaling || enableCombinedSlipFriction || enableModernLowSpeedTire)
                state.shearDispWorld = ProjectOnPlane(state.shearDispWorld, contactNormal);

            float speedFloor = enableModernLowSpeedTire
                ? Mathf.Max(vEps, lowSpeedRelaxSpeedFloor)
                : vEps;
            float speed = Mathf.Max(slipSpeed, speedFloor);
            float relaxationTime = Mathf.Max(1e-4f, relaxLength / speed);

            float leakScale = 1f;
            if (enableModernLowSpeedTire)
            {
                float leakSpeed = Mathf.Max(1e-4f, lowSpeedCreepSpeed);
                leakScale = Mathf.Clamp01(slipSpeed / leakSpeed);
            }

            Vector3 displacementRate = slipVelocity
                                     - state.shearDispWorld * (leakScale / relaxationTime);
            state.shearDispWorld += displacementRate * fixedDeltaTime;

            if (maxShearDisp > 1e-5f)
            {
                float displacement = state.shearDispWorld.magnitude;
                if (displacement > maxShearDisp)
                    state.shearDispWorld *= maxShearDisp / displacement;
            }

            Vector3 rawForce = -shearK * state.shearDispWorld;
            float damping = shearC;
            if (enableModernLowSpeedTire)
                damping = Mathf.Max(damping, lowSpeedShearDampingC);
            if (damping > 0f) rawForce += -damping * displacementRate;

            bool staticZone = slipSpeed < Mathf.Max(1e-3f, vStatic);
            if (enableModernLowSpeedTire)
            {
                Vector3 creepForce = BuildLowSpeedCreepForce(slipVelocity, forward, side, normalLoad);
                float blendSpeed = Mathf.Max(1e-4f, lowSpeedCreepBlend);
                float creepBlend = 1f - Mathf.Clamp01(slipSpeed / blendSpeed);
                creepBlend = creepBlend * creepBlend * (3f - 2f * creepBlend);
                rawForce += creepForce * creepBlend;
            }

            if (enableCombinedSlipFriction && side.sqrMagnitude > 1e-6f)
            {
                float longScale = Mathf.Max(0f, longitudinalGripScale);
                float latScale = Mathf.Max(0f, lateralGripScale);
                float scale;
                if (staticZone)
                {
                    scale = GetCombinedFrictionScale(
                        rawForce, forward, side,
                        muStatic * longScale * normalLoad,
                        muStatic * latScale * normalLoad);
                    if (scale < 0.9999f)
                        scale = GetCombinedFrictionScale(
                            rawForce, forward, side,
                            muKinetic * longScale * normalLoad,
                            muKinetic * latScale * normalLoad);
                }
                else
                {
                    scale = GetCombinedFrictionScale(
                        rawForce, forward, side,
                        muKinetic * longScale * normalLoad,
                        muKinetic * latScale * normalLoad);
                }

                if (scale < 0.9999f)
                {
                    rawForce *= scale;
                    state.shearDispWorld *= scale;
                }
            }
            else
            {
                float maximumStaticForce = muStatic * normalLoad;
                float maximumKineticForce = muKinetic * normalLoad;
                float forceMagnitude = rawForce.magnitude;
                if (staticZone)
                {
                    if (forceMagnitude > maximumStaticForce && forceMagnitude > 1e-6f)
                    {
                        rawForce *= maximumKineticForce / forceMagnitude;
                        state.shearDispWorld = -rawForce / Mathf.Max(1e-6f, shearK);
                    }
                }
                else if (forceMagnitude > maximumKineticForce && forceMagnitude > 1e-6f)
                {
                    rawForce *= maximumKineticForce / forceMagnitude;
                    state.shearDispWorld = -rawForce / Mathf.Max(1e-6f, shearK);
                }
            }

            if (forceFilterTau > 1e-5f)
            {
                float filterAlpha = fixedDeltaTime / (forceFilterTau + fixedDeltaTime);
                state.FtireFiltered = Vector3.Lerp(state.FtireFiltered, rawForce, filterAlpha);
                tireForce = state.FtireFiltered;
            }
            else
            {
                state.FtireFiltered = rawForce;
                tireForce = rawForce;
            }

            if (enableSinglePassLoadScaling)
            {
                float postMu = staticZone ? muStatic : muKinetic;
                if (enableCombinedSlipFriction && side.sqrMagnitude > 1e-6f)
                {
                    tireForce = ClampCombinedTireForce(
                        tireForce, forward, side,
                        postMu * Mathf.Max(0f, longitudinalGripScale) * normalLoad,
                        postMu * Mathf.Max(0f, lateralGripScale) * normalLoad);
                }
                else
                {
                    tireForce = LimitVectorMagnitude(tireForce, postMu * normalLoad);
                }
            }
        }
        else if (slipSpeed > 1e-5f)
        {
            if (enableCombinedSlipFriction && side.sqrMagnitude > 1e-6f)
            {
                tireForce = BuildCombinedKineticFriction(
                    slipVelocity, forward, side,
                    muKinetic * Mathf.Max(0f, longitudinalGripScale) * normalLoad,
                    muKinetic * Mathf.Max(0f, lateralGripScale) * normalLoad);
            }
            else
            {
                tireForce = -slipVelocity / slipSpeed * (muKinetic * normalLoad);
            }
        }

        if (!enableSinglePassLoadScaling)
        {
            tireForce *= gate;
            tireForce *= sampleWeight;
        }

        if (tireForce.sqrMagnitude > 1e-10f)
            ApplyTireForce(tireForce, sample.p, groundBody, wheelAxis);
        return tireForce;
    }

    private void ApplyTireForce(
        Vector3 force,
        Vector3 point,
        Rigidbody groundRb,
        Vector3 wheelAxisWorld)
    {
        if (force.sqrMagnitude <= 1e-12f) return;

        bool useDecoupled = enableDecoupledTireForceApplication && decoupleTireForceAndTorque;
        if (useDecoupled)
        {
            Rigidbody.AddForce(force, ForceMode.Force);
            if (wheelAxisWorld.sqrMagnitude > 1e-10f)
            {
                wheelAxisWorld.Normalize();
                Vector3 tau = Vector3.Cross(point - Rigidbody.worldCenterOfMass, force);
                float spinTau = Vector3.Dot(tau, wheelAxisWorld);
                if (Mathf.Abs(spinTau) > 1e-6f)
                    Rigidbody.AddTorque(wheelAxisWorld * spinTau, ForceMode.Force);
            }
        }
        else
        {
            Rigidbody.AddForceAtPosition(force, point, ForceMode.Force);
        }

        if (groundRb != null)
            groundRb.AddForceAtPosition(-force, point, ForceMode.Force);
    }

    private Vector3 LimitVectorMagnitude(Vector3 v, float maxMagnitude)
    {
        if (maxMagnitude <= 0f) return Vector3.zero;
        float sqr = v.sqrMagnitude;
        float maxSqr = maxMagnitude * maxMagnitude;
        if (sqr <= maxSqr || sqr <= 1e-12f) return v;
        return v * (maxMagnitude / Mathf.Sqrt(sqr));
    }

    private float GetCombinedFrictionScale(Vector3 force, Vector3 forward, Vector3 side, float maxLong, float maxSide)
    {
        if (force.sqrMagnitude <= 1e-12f) return 1f;
        float fx = Vector3.Dot(force, forward);
        float fy = Vector3.Dot(force, side);
        float usage = 0f;

        if (maxLong > 1e-6f) usage += fx * fx / (maxLong * maxLong);
        else if (Mathf.Abs(fx) > 1e-6f) return 0f;
        if (maxSide > 1e-6f) usage += fy * fy / (maxSide * maxSide);
        else if (Mathf.Abs(fy) > 1e-6f) return 0f;

        return usage <= 1f ? 1f : 1f / Mathf.Sqrt(usage);
    }

    private Vector3 ClampCombinedTireForce(Vector3 force, Vector3 forward, Vector3 side, float maxLong, float maxSide)
    {
        float fx = Vector3.Dot(force, forward);
        float fy = Vector3.Dot(force, side);
        Vector3 tangentForce = forward * fx + side * fy;
        return tangentForce * GetCombinedFrictionScale(tangentForce, forward, side, maxLong, maxSide);
    }

    private Vector3 BuildCombinedKineticFriction(Vector3 vSlip, Vector3 forward, Vector3 side, float maxLong, float maxSide)
    {
        Vector3 demand = Vector3.zero;
        float vx = Vector3.Dot(vSlip, forward);
        if (Mathf.Abs(vx) > 1e-5f && maxLong > 0f)
            demand += -Mathf.Sign(vx) * maxLong * forward;
        float vy = Vector3.Dot(vSlip, side);
        if (Mathf.Abs(vy) > 1e-5f && maxSide > 0f)
            demand += -Mathf.Sign(vy) * maxSide * side;
        return ClampCombinedTireForce(demand, forward, side, maxLong, maxSide);
    }

    private Vector3 BuildLowSpeedCreepForce(Vector3 vSlip, Vector3 forward, Vector3 side, float normalLoad)
    {
        if (!enableModernLowSpeedTire || normalLoad <= 1e-6f) return Vector3.zero;
        float creepV = Mathf.Max(1e-4f, lowSpeedCreepSpeed);

        if (enableCombinedSlipFriction && side.sqrMagnitude > 1e-6f)
        {
            float maxLong = muStatic * Mathf.Max(0f, longitudinalGripScale) * normalLoad;
            float maxSide = muStatic * Mathf.Max(0f, lateralGripScale) * normalLoad;
            Vector3 demand = Vector3.zero;
            float vx = Vector3.Dot(vSlip, forward);
            if (Mathf.Abs(vx) > 1e-6f && maxLong > 0f)
                demand += -Mathf.Clamp(vx / creepV, -1f, 1f) * maxLong * forward;
            float vy = Vector3.Dot(vSlip, side);
            if (Mathf.Abs(vy) > 1e-6f && maxSide > 0f)
                demand += -Mathf.Clamp(vy / creepV, -1f, 1f) * maxSide * side;
            return ClampCombinedTireForce(demand, forward, side, maxLong, maxSide);
        }

        float speed = vSlip.magnitude;
        if (speed <= 1e-6f) return Vector3.zero;
        float demandMagnitude = muStatic * normalLoad * Mathf.Clamp01(speed / creepV);
        return -vSlip / speed * demandMagnitude;
    }

    private void ApplyAxleSpinStabilization(float normalLoad, float radius, Vector3 wheelAxisWorld)
    {
        if (!enableModernLowSpeedTire || !HasRigidbody) return;
        if (wheelAxisWorld.sqrMagnitude < 1e-10f) return;
        wheelAxisWorld.Normalize();

        float omega = Vector3.Dot(Rigidbody.angularVelocity, wheelAxisWorld);
        if (Mathf.Abs(omega) <= 1e-5f) return;
        bool hasLoad = normalLoad > 1e-6f && radius > 1e-6f;
        float dampingK = hasLoad ? axleSpinDampingK : axleAirDampingK;
        if (dampingK <= 0f) return;

        float tau = -omega * dampingK;
        if (hasLoad)
        {
            float tauLimit = Mathf.Max(0f, muStatic) * normalLoad * radius;
            if (tauLimit > 1e-6f) tau = Mathf.Clamp(tau, -tauLimit, tauLimit);
        }

        float inertia = GetInertiaAroundWorldAxis(wheelAxisWorld);
        float dt = Mathf.Max(1e-5f, Time.fixedDeltaTime);
        if (inertia > 1e-6f)
        {
            float stopTorque = Mathf.Abs(omega) * inertia / dt;
            if (stopTorque > 1e-6f) tau = Mathf.Clamp(tau, -stopTorque, stopTorque);
        }

        if (Mathf.Abs(tau) > 1e-6f)
            Rigidbody.AddTorque(wheelAxisWorld * tau, ForceMode.Force);
    }

    private float GetInertiaAroundWorldAxis(Vector3 axisWorld)
    {
        if (!HasRigidbody || axisWorld.sqrMagnitude < 1e-10f) return 0f;
        axisWorld.Normalize();
        Quaternion principalToWorld = Rigidbody.rotation * Rigidbody.inertiaTensorRotation;
        Vector3 axisPrincipal = Quaternion.Inverse(principalToWorld) * axisWorld;
        if (axisPrincipal.sqrMagnitude > 1e-10f) axisPrincipal.Normalize();
        Vector3 inertia = Rigidbody.inertiaTensor;
        return Mathf.Max(0f, inertia.x) * axisPrincipal.x * axisPrincipal.x
             + Mathf.Max(0f, inertia.y) * axisPrincipal.y * axisPrincipal.y
             + Mathf.Max(0f, inertia.z) * axisPrincipal.z * axisPrincipal.z;
    }
}
