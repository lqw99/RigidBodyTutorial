#import "@preview/physica:0.9.8": *

#show: super-T-as-transpose


== step 3 compute contact Jacobian

boxed-LCP(BLCP)

- `void Contact::computeContactFrame()` compute the contact frame that includes the normal $n$ and the two tangent directions, $t$ and $b$ and the normal is already given by collsion detection, in code, two tangent directions are $t_1$ and $t_2$

- `void Contact::computeJacobian()` compute the constraint Jacobian based on

for each contact $i$, the local frame $vb(n),vb(t)_1,vb(t)_2$ and the contact vector arms
$ vb(r)_A = vb(p) - vb(x)_A quad vb(r)_B = vb(p) - vb(x)_B $,

relative contact point velocity is
$
  Delta vb(v) = vb(v)_B + omega_B times vb(r)_B - (vb(v)_A + omega_A times vb(r)_A)
$
project it into local frame
$
  Delta vb(v) = vec(Delta v_n, Delta v_(t_1), Delta v_(t_2)) = vb(J) vec(vb(v)_A, omega_A, vb(v_B), omega_B) = mat(vb(J)_A, vb(J)_B) vec(vb(v)_A, omega_A, vb(v_B), omega_B)
$


constrained equations of motion based on BLCP
$
  vb(M) vb(u)^+ = vb(J)^T vb(lambda)^+ + vb(M) vb(u) + h vb(f) \
  vb(J) vb(u)^+ = vb(v) \
  => underbrace(vb(J) vb(M) vb(J)^T, vb(A)) underbrace(vb(lambda)^+, vb(x)) + underbrace(vb(J) vb(u) + vb(J) vb(M)^(-1) h vb(f), vb(b)) = vb(v) \
$
where $vb(v)$ is relative velocity in contact frame at contact points



