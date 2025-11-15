using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// Check this out we can require components be on a game object!
[RequireComponent(typeof(MeshFilter))]

public class BParticleSimMesh : MonoBehaviour
{
    public struct BSpring
    {
        public float kd;                        // damping coefficient
        public float ks;                        // spring coefficient
        public float restLength;                // rest length of this spring
        public int attachedParticle;            // index of the attached other particle (use me wisely to avoid doubling springs and sprign calculations)
    }

    public struct BContactSpring
    {
        public float kd;                        // damping coefficient
        public float ks;                        // spring coefficient
        public float restLength;                // rest length of this spring (think about this ... may not even be needed o_0
        public Vector3 attachPoint;             // the attached point on the contact surface
    }

    public struct BParticle
    {
        public Vector3 position;                // position information
        public Vector3 velocity;                // velocity information
        public float mass;                      // mass information
        public BContactSpring contactSpring;    // Special spring for contact forces
        public bool attachedToContact;          // is thi sparticle currently attached to a contact (ground plane contact)
        public List<BSpring> attachedSprings;   // all attached springs, as a list in case we want to modify later fast
        public Vector3 currentForces;           // accumulate forces here on each step        
    }

    public struct BPlane
    {
        public Vector3 position;                // plane position
        public Vector3 normal;                  // plane normal
    }

    public float contactSpringKS = 1000.0f;     // contact spring coefficient with default 1000
    public float contactSpringKD = 20.0f;       // contact spring daming coefficient with default 20

    public float defaultSpringKS = 100.0f;      // default spring coefficient with default 100
    public float defaultSpringKD = 1.0f;        // default spring daming coefficient with default 1

    public bool debugRender = false;            // To render or not to render


    /*** 
     * I've given you all of the above to get you started
     * Here you need to publicly provide the:
     * - the ground plane transform (Transform)
     * - handlePlaneCollisions flag (bool)
     * - particle mass (float)
     * - useGravity flag (bool)
     * - gravity value (Vector3)
     * Here you need to privately provide the:
     * - Mesh (Mesh)
     * - array of particles (BParticle[])
     * - the plane (BPlane)
     ***/

    public Transform groundPlaneTransform;
    public bool handlePlaneCollisions = true;
    public float particleMass = 1.0f;
    public bool useGravity = true;
    public Vector3 gravity = new Vector3(0, -9.8f, 0);


    private Mesh mesh;
    private BParticle[] particles;
    private BPlane plane;	


    /// <summary>
    /// Init everything
    /// HINT: in particular you should probbaly handle the mesh, init all the particles, and the ground plane
    /// HINT 2: I'd for organization sake put the init particles and plane stuff in respective functions
    /// HINT 3: Note that mesh vertices when accessed from the mesh filter are in local coordinates.
    ///         This script will be on the object with the mesh filter, so you can use the functions
    ///         transform.TransformPoint and transform.InverseTransformPoint accordingly 
    ///         (you need to operate on world coordinates, and render in local)
    /// HINT 4: the idea here is to make a mathematical particle object for each vertex in the mesh, then connect
    ///         each particle to every other particle. Be careful not to double your springs! There is a simple
    ///         inner loop approach you can do such that you attached exactly one spring to each particle pair
    ///         on initialization. Then when updating you need to remember a particular trick about the spring forces
    ///         generated between particles. 
    /// </summary>
    void Start()
    {
     mesh = GetComponent<MeshFilter>().mesh;
     InitParticles();
     InitPlane();

    }



    /*** BIG HINT: My solution code has as least the following functions
     * InitParticles()
     * InitPlane()
     * UpdateMesh() (remember the hint above regarding global and local coords)
     * ResetParticleForces()
     * ...
     ***/

    void InitParticles()
    {
        Vector3[] vertsLocal = mesh.vertices;
        int count = vertsLocal.Length;

        particles = new BParticle[count];

        // Creating  particles
        for (int i = 0; i < count; i++)
        {
            particles[i].position = transform.TransformPoint(vertsLocal[i]); // to world space
            particles[i].velocity = Vector3.zero;
            particles[i].mass = particleMass;
            particles[i].attachedSprings = new List<BSpring>();
            particles[i].currentForces = Vector3.zero;
            particles[i].attachedToContact = false;
        }

        // Creating  springs between every pair (i < j to avoid duplicates)
        for (int i = 0; i < count; i++)
        {
            for (int j = i + 1; j < count; j++)
            {
                Vector3 diff = particles[i].position - particles[j].position;
                float restLen = diff.magnitude;

                BSpring s = new BSpring();
                s.ks = defaultSpringKS;
                s.kd = defaultSpringKD;
                s.restLength = restLen;
                s.attachedParticle = j;

                particles[i].attachedSprings.Add(s);
            }
        }
    }

    void InitPlane()
    {
        if (groundPlaneTransform != null)
        {
            plane.position = groundPlaneTransform.position;
            plane.normal = groundPlaneTransform.up.normalized;
        }
        else
        {
            // Fallback: y=0, normal up
            plane.position = Vector3.zero;
            plane.normal = Vector3.up;
        }
    }

    void ResetParticleForces()
    {
        for (int i = 0; i < particles.Length; i++)
        {
            particles[i].currentForces = Vector3.zero;

            if (useGravity)
            {
                particles[i].currentForces += gravity * particles[i].mass;
            }
        }
    }
    
    void ComputeParticleSpringForces()
    {
        int n = particles.Length;

        for (int i = 0; i < n; i++)
        {
            int springCount = particles[i].attachedSprings.Count;

            for (int s = 0; s < springCount; s++)
            {
                BSpring spring = particles[i].attachedSprings[s];
                int j = spring.attachedParticle;

                Vector3 xi = particles[i].position;
                Vector3 xj = particles[j].position;
                Vector3 vi = particles[i].velocity;
                Vector3 vj = particles[j].velocity;

                Vector3 diff = xi - xj;
                float dist = diff.magnitude;
                if (dist <= Mathf.Epsilon) continue;

                Vector3 n_ij = diff / dist; // unit direction i->j

                // Spring term =  ks (l - |xi-xj|) n
                float stretch = spring.restLength - dist;
                Vector3 Fspring = spring.ks * stretch * n_ij;

                // damping term =  -kd( (vi - vj)·n ) n
                float relVelAlongN = Vector3.Dot(vi - vj, n_ij);
                Vector3 Fdamp = -spring.kd * relVelAlongN * n_ij;

                Vector3 F = Fspring + Fdamp;

                // Apply equal and opposite forces
                particles[i].currentForces += F;
                particles[j].currentForces -= F;
            }
        }
    }


    void UpdateContactSprings()
    {
        if (!handlePlaneCollisions) return;

        for (int i = 0; i < particles.Length; i++)
        {
            Vector3 xp = particles[i].position;
            float dist = Vector3.Dot(xp - plane.position, plane.normal);

            bool penetrating = dist < 0.0f;

            if (penetrating && !particles[i].attachedToContact)
            {
                particles[i].attachedToContact = true;

                BContactSpring cs = new BContactSpring();
                cs.ks = contactSpringKS;
                cs.kd = contactSpringKD;
                cs.restLength = 0.0f;

                // nearest point on plane at moment of detection
                cs.attachPoint = xp - dist * plane.normal;

                particles[i].contactSpring = cs;
            }
            else if (!penetrating && particles[i].attachedToContact)
            {
                particles[i].attachedToContact = false;
            }
        }
    }

    void ComputeContactForces()
    {
        for (int i = 0; i < particles.Length; i++)
        {
            if (!particles[i].attachedToContact) continue;

            BContactSpring cs = particles[i].contactSpring;

            Vector3 xp = particles[i].position;
            Vector3 vp = particles[i].velocity;

            Vector3 diff = xp - cs.attachPoint;
            float depth = Vector3.Dot(diff, plane.normal); 

            // spring term
            Vector3 Fspring = -cs.ks * depth * plane.normal;

            // damping term
            Vector3 Fdamp = -cs.kd * vp;

            Vector3 F = Fspring + Fdamp;

            particles[i].currentForces += F;
        }
    }
    
    void Integrate(float dt)
    {
        for (int i = 0; i < particles.Length; i++)
        {
            Vector3 a = particles[i].currentForces / particles[i].mass;

            particles[i].velocity += a * dt;
            particles[i].position += particles[i].velocity * dt;
        }
    }
    
    void UpdateMesh()
    {
        Vector3[] vertsLocal = mesh.vertices;

        for (int i = 0; i < vertsLocal.Length; i++)
        {
            vertsLocal[i] = transform.InverseTransformPoint(particles[i].position);
        }

        mesh.vertices = vertsLocal;
        mesh.RecalculateBounds();
        mesh.RecalculateNormals();
    }
       
    /// <summary>
    /// Draw a frame with some helper debug render code
    /// </summary>
    public void Update()
    {

        float dt = Time.deltaTime;

        if (particles == null || particles.Length == 0) 
        return;

        ResetParticleForces();
        UpdateContactSprings();
        ComputeParticleSpringForces();
        ComputeContactForces();
        Integrate(dt);
        UpdateMesh();


        if (debugRender)
        {
            int particleCount = particles.Length;
            for (int i = 0; i < particleCount; i++)
            {
                Debug.DrawLine(particles[i].position, particles[i].position + particles[i].currentForces* 0.01f, Color.blue);

                int springCount = particles[i].attachedSprings.Count;
                for (int j = 0; j < springCount; j++)
                {   
                    Debug.DrawLine(particles[i].position, particles[particles[i].attachedSprings[j].attachedParticle].position, Color.red);
                }
            }
        }
    }
}
