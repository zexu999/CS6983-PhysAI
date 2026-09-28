// AI USAGE: This file was written with assistance from Claude (Anthropic AI).
// Purpose: small helper so the project compiles on both Unity 2022 LTS and Unity 6
// (Unity 6 renamed Rigidbody.drag -> linearDamping, velocity -> linearVelocity,
//  and PhysicMaterial -> PhysicsMaterial).
using UnityEngine;

public static class PhysicsCompat
{
    public static void SetDamping(Rigidbody rb, float linear, float angular)
    {
#if UNITY_6000_0_OR_NEWER
        rb.linearDamping = linear;
        rb.angularDamping = angular;
#else
        rb.drag = linear;
        rb.angularDrag = angular;
#endif
    }

    public static Vector3 GetVelocity(Rigidbody rb)
    {
#if UNITY_6000_0_OR_NEWER
        return rb.linearVelocity;
#else
        return rb.velocity;
#endif
    }

    public static void SetVelocity(Rigidbody rb, Vector3 v)
    {
#if UNITY_6000_0_OR_NEWER
        rb.linearVelocity = v;
#else
        rb.velocity = v;
#endif
    }

    /// <summary>Creates a physics material with the given friction/bounce and assigns it to the collider.</summary>
    public static void ApplyMaterial(Collider col, string name, float dynamicFriction, float staticFriction, float bounciness)
    {
#if UNITY_6000_0_OR_NEWER
        var m = new PhysicsMaterial(name);
        m.frictionCombine = PhysicsMaterialCombine.Average;
        m.bounceCombine = PhysicsMaterialCombine.Minimum;
#else
        var m = new PhysicMaterial(name);
        m.frictionCombine = PhysicMaterialCombine.Average;
        m.bounceCombine = PhysicMaterialCombine.Minimum;
#endif
        m.dynamicFriction = dynamicFriction;
        m.staticFriction = staticFriction;
        m.bounciness = bounciness;
        col.sharedMaterial = m;
    }
}
