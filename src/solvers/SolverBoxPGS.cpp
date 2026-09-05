#include "solvers/SolverBoxPGS.h"

#include "Eigen/Core"
#include "contact/Contact.h"
#include "rigidbody/RigidBody.h"
#include "rigidbody/RigidBodySystem.h"

#include <Eigen/Dense>
#include <Eigen/LU>

using Vec6f = Eigen::Vector<float, 6>;

SolverBoxPGS::SolverBoxPGS(RigidBodySystem *_rigidBodySystem)
    : Solver(_rigidBodySystem) {}

bool check_tangent_impulse(float lambda_t, float lambda_n, float v_t, float mu,
                           int state) {
  float lower = -mu * lambda_n;
  float upper = mu * lambda_n;

  float eps = 1e-4;

  switch (state) {
  case 0: // free in tangent
    if (lambda_t < lower - eps || lambda_t > upper + eps)
      return false;
    if (std::abs(v_t) > eps)
      return false;
    break;
  case -1:
    if (std::abs(lambda_t - lower) > eps)
      return false;
    if (v_t < -eps)
      return false;

    break;
  case 1:
    if (std::abs(lambda_t - upper) > eps)
      return false;
    if (v_t > eps)
      return false;
    break;
  default:
    break;
  }
  return true;
}

bool solve_contact(const Eigen::Matrix3f &A, const Eigen::Vector3f &x,
                   Eigen::VectorXf &lambda, float mu) {
  constexpr float eps = 1e-4;

  // separated contact
  if (x(0) >= -eps) {
    // std::cout << x << std::endl;
    lambda.setZero();
    return true;
  }

  const std::array<int, 3> tangent_states = {-1, 0, 1};
  for (auto s1 : tangent_states) {
    for (auto s2 : tangent_states) {
      Eigen::Matrix3f K = Eigen::Matrix3f::Zero();
      Eigen::Vector3f b = Eigen::Vector3f::Zero();
      // normal active, v_n = 0
      K.row(0) = A.row(0);
      b(0) = -x(0);

      if (s1 == 0) {
        // no tangent slip, v_t1 = 0
        K.row(1) = A.row(1);
        b(1) = -x(1);
      } else if (s1 == -1) {
        // tangent slip exists, lambda_t1 = -mu lambda_n
        K(1, 0) = mu;
        K(1, 1) = 1.0;
      } else {
        // tangent slip exists, lambda_t1 = mu lambda_n
        K(1, 0) = -mu;
        K(1, 1) = 1.0;
      }

      if (s2 == 0) {
        // no tangent slip, v_t2 = 0
        K.row(2) = A.row(2);
        b(2) = -x(2);
      } else if (s2 == -1) {
        // tangent slip exists, lambda_t1 = -mu lambda_n
        K(2, 0) = mu;
        K(2, 2) = 1.0;
      } else {
        // tangent slip exists, lambda_t1 = mu lambda_n
        K(2, 0) = -mu;
        K(2, 2) = 1.0;
      }

      // solve K lambda + b = 0
      Eigen::FullPivLU<Eigen::Matrix3f> lu(K);

      if (!lu.isInvertible())
        continue;

      Eigen::Vector3f xx = lu.solve(b);

      if (xx(0) < -eps) // error in normal contact impulse
        continue;

      Eigen::Vector3f v = A * xx + x;
      // no normal velocity when normal contact is active
      if (std::abs(v(0)) > eps)
        continue;

      if (!check_tangent_impulse(xx(1), xx(0), v(1), mu, s1)) {
        continue;
      }
      if (!check_tangent_impulse(xx(2), xx(0), v(2), mu, s2)) {
        continue;
      }
      lambda = xx;

      return true;
    }
  }
  return false;
}

void SolverBoxPGS::solve(float h) {
  // TODO Implement a PGS method that solves for the
  //      contact constraint forces in @a rigidBodySystem.
  //      Assume a boxed LCP formulation.
  //
  std::vector<Contact *> &contacts = m_rigidBodySystem->getContacts();
  const int numContacts = contacts.size();

  // Build array of 3x3 diagonal matrices, one for each contact.
  //

  if (numContacts > 0) {
    std::vector<Eigen::Matrix3f> Acontactii(numContacts);
    // TODO Compute the right-hand side vector : b = -gamma*phi/h - J*vel -
    // dt*JMinvJT*force
    //
    float gamma = 0.2;
    std::vector<Eigen::Vector3f> bs(numContacts);
    for (int i = 0; i < numContacts; ++i) {
      bs[i] = Eigen::Vector3f::Zero();

      auto c = contacts[i];

      bs[i] += gamma * c->phi / h;
      auto v0 = c->body0->xdot;
      auto w0 = c->body0->omega;
      auto v1 = c->body1->xdot;
      auto w1 = c->body1->omega;
      Vec6f u0, u1;
      u0 << v0, w0;
      u1 << v1, w1;

      bs[i] += (c->J0 * u0 + c->J1 * u1);
      auto f0 = c->body0->f;
      auto tau0 = c->body0->tau;
      auto f1 = c->body1->f;
      auto tau1 = c->body1->tau;

      Vec6f ff0, ff1;
      ff0 << f0, tau0;
      ff1 << f1, tau1;
      bs[i] += h * (c->J0Minv * ff0 + c->J1Minv * ff1);

      // TODO Compute the diagonal term : Aii = J0*Minv0*J0^T + J1*Minv1*J1^T
      //
      Acontactii[i] =
          c->J0Minv * c->J0.transpose() + c->J1Minv * c->J1.transpose();
    }

    // PGS main loop.
    // There is no convergence test here. Simply stop after @a maxIter
    // iterations.
    //
    for (int iter = 0; iter < m_maxIter; ++iter) {
      // TODO For each contact, compute an updated value of contacts[i]->lambda
      //      using matrix-free pseudo-code provided in the course appendix.
      //
      for (int i = 0; i < numContacts; ++i) {
        // TODO initialize current solution as x = b(i)
        //
        Eigen::Vector3f b = bs[i];

        // TODO Loop over all other contacts involving c->body0
        //      and accumulate : x -= (J0*Minv0*Jother^T) * lambda_other
        //

        auto c = contacts[i];
        auto body0 = c->body0;
        if (!body0->fixed) {
          for (auto c_other : body0->contacts) {
            if (c_other != c) {
              auto lambda_other = c_other->lambda;
              JBlock J_other;
              if (c_other->body0 == body0) {
                J_other = c_other->J0;
              } else {
                J_other = c_other->J1;
              }
              b += (c->J0Minv * J_other.transpose()) * lambda_other;
            }
          }
        }

        // // TODO Loop over all other contacts involving c->body1
        // //      and accumulate : x -= (J0*Minv0*Jother^T) * lambda_other
        // //
        auto body1 = c->body1;
        if (!body1->fixed) {
          for (auto c_other : body1->contacts) {
            if (c_other != c) {
              auto lambda_other = c_other->lambda;
              JBlock J_other;
              if (c_other->body0 == body1) {
                J_other = c_other->J0;
              } else {
                J_other = c_other->J1;
              }
              b += (c->J1Minv * J_other.transpose()) * lambda_other;
            }
          }
        }

        // TODO Update lambda by solving the sub-problem : Aii *
        // contacts[i].lambda = x
        //      For non-interpenetration constraints, lambda is non-negative and
        //      should be clamped to the range (0... infinity). Otherwise
        //      friction constraints should be clamped to the range
        //      (-mu*lambda_n, mu*lambda_n) where lambda_n is the impulse of the
        //      corresponding non-interpentration constraint.
        //

        // direct enumeration

        auto Aii = Acontactii[i];
        bool is_solved = solve_contact(Aii, b, c->lambda, c->mu);
      }
    }
  }
}
