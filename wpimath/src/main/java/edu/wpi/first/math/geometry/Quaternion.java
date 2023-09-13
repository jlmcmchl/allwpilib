// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.geometry;

import com.fasterxml.jackson.annotation.JsonAutoDetect;
import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.numbers.N3;
import java.util.Objects;
import java.util.Random;

@JsonIgnoreProperties(ignoreUnknown = true)
@JsonAutoDetect(getterVisibility = JsonAutoDetect.Visibility.NONE)
public class Quaternion implements Interpolatable<Quaternion> {
  // Scalar r in versor form
  private final double m_w;

  // Vector v in versor form
  private final double m_x;
  private final double m_y;
  private final double m_z;

  /** Constructs a quaternion with a default angle of 0 degrees. */
  public Quaternion() {
    m_w = 1.0;
    m_x = 0.0;
    m_y = 0.0;
    m_z = 0.0;
  }

  /**
   * Constructs a quaternion with the given components.
   *
   * @param w W component of the quaternion.
   * @param x X component of the quaternion.
   * @param y Y component of the quaternion.
   * @param z Z component of the quaternion.
   */
  @JsonCreator
  public Quaternion(
      @JsonProperty(required = true, value = "W") double w,
      @JsonProperty(required = true, value = "X") double x,
      @JsonProperty(required = true, value = "Y") double y,
      @JsonProperty(required = true, value = "Z") double z) {
    m_w = w;
    m_x = x;
    m_y = y;
    m_z = z;
  }

  /**
   * Adds another quaternion to this quaternion entrywise.
   * 
   * @param other The other quaternion.
   * @return The quaternion sum.
   */
  public Quaternion plus(Quaternion other) {
    return new Quaternion(getW() + other.getW(), getX() + other.getX(), getY() + other.getY(), getZ() + other.getZ());
  }

  /**
   * Multiplies with a scalar.
   * 
   * @param scalar The value to scale each component by.
   * @return The scaled quaternion.
   */
  public Quaternion times(double scalar) {
    return new Quaternion(getW() * scalar, getX() * scalar, getY() * scalar, getZ() * scalar);
  }

  /**
   * Multiply with another quaternion.
   *
   * @param other The other quaternion.
   * @return The quaternion product.
   */
  public Quaternion times(Quaternion other) {
    // https://en.wikipedia.org/wiki/Quaternion#Scalar_and_vector_parts
    final var r1 = m_w;
    final var r2 = other.m_w;

    // v₁ ⋅ v₂
    double dot = m_x * other.m_x + m_y * other.m_y + m_z * other.m_z;

    // v₁ x v₂
    double cross_x = m_y * other.m_z - other.m_y * m_z;
    double cross_y = other.m_x * m_z - m_x * other.m_z;
    double cross_z = m_x * other.m_y - other.m_x * m_y;

    return new Quaternion(
        // r = r₁r₂ − v₁ ⋅ v₂
        r1 * r2 - dot,
        // v = r₁v₂ + r₂v₁ + v₁ x v₂
        r1 * other.m_x + r2 * m_x + cross_x,
        r1 * other.m_y + r2 * m_y + cross_y,
        r1 * other.m_z + r2 * m_z + cross_z);
  }

  @Override
  public String toString() {
    return String.format("Quaternion(%s, %s, %s, %s)", getW(), getX(), getY(), getZ());
  }

  /**
   * Checks equality between this Quaternion and another object.
   *
   * @param obj The other object.
   * @return Whether the two objects are equal or not.
   */
  @Override
  public boolean equals(Object obj) {
    if (obj instanceof Quaternion) {
      var other = (Quaternion) obj;

      return Math.abs(
              getW() * other.getW()
                  + getX() * other.getX()
                  + getY() * other.getY()
                  + getZ() * other.getZ())
          > 1.0 - 1E-9;
    }
    return false;
  }

  @Override
  public int hashCode() {
    return Objects.hash(m_w, m_x, m_y, m_z);
  }
  
  /**
   * Returns the conjugate of the quaternion.
   * 
   * @return The conjugate quaternion.
   */
  public Quaternion conjugate() {
    return new Quaternion(getW(), -getX(), -getY(), -getZ());
  }

  /**
   * Returns the inverse of the quaternion.
   *
   * @return The inverse quaternion.
   */
  public Quaternion inverse() {
    var norm = norm();
    return conjugate().times(1 / (norm * norm));
  }

  /**
   * Calculates the L2 norm of the quaternion.
   * 
   * @return The L2 norm.
   */
  public double norm() {
    return Math.sqrt(getW() * getW() + getX() * getX() + getY() * getY() + getZ() * getZ());
  }

  /**
   * Normalizes the quaternion.
   *
   * @return The normalized quaternion.
   */
  public Quaternion normalize() {
    double norm = norm();
    if (norm == 0.0) {
      return new Quaternion();
    } else {
      return new Quaternion(getW() / norm, getX() / norm, getY() / norm, getZ() / norm);
    }
  }

  
  public Quaternion exp(Quaternion adjustment) {
    return adjustment.exp().times(this);
  }

  /**
   * Matrix exponential of a quaternion
   * source: https://en.wikipedia.org/wiki/Quaternion#Exponential,_logarithm,_and_power_functions
   *
   * @return The Matrix exponential of this quaternion.
   */
  public Quaternion exp() {
    // q = s(scalar) + v(vector)
    // exp(s)
    var scalar = Math.exp(m_w);

    // ||v||
    var axial_magnitude = Math.sqrt(getX() * getX() + getY() * getY() + getZ() * getZ());
    // cos(||v||)
    var cosine = Math.cos(axial_magnitude);
    // sin(||v||)
    var sine = Math.sin(axial_magnitude);
    
    // exp(s) * (cos(||v||) + v / ||v|| * sin(||v||))
    return new Quaternion(
        cosine, 
        getX() / axial_magnitude * sine, 
        getY() / axial_magnitude * sine, 
        getZ() / axial_magnitude * sine)
      .times(scalar);
  }

  /**
   * Inverse Matrix exponential (logarithm) of a quaternion.
   * 
   * <p> This method is intended for use with unit quaternions. In this case, this method returns the rotation vector which transforms this quaternion into @param end via this.exp(rvec) <p>
   * 
   * @return The logarithm of this quaternion.
   */
  public Quaternion log(Quaternion end) {
    return end.times(inverse()).log();
  }

  /**
   * Inverse Matrix exponential (logarithm) of a quaternion.
   * 
   * <p> source: https://en.wikipedia.org/wiki/Quaternion#Exponential,_logarithm,_and_power_functions <p>
   * 
   * <p> For unit quaternions, this is equivalent to @see Quaternion#toRotationVector(). <p>
   * 
   * @return The logarithm of this quaternion.
   */
  public Quaternion log() {
    // q = s(scalar) + v(vector)

    // ||q||
    var norm = norm();

    // ln(||q||)
    var scalar = Math.log(norm);

    // ||v||
    var axial_magnitude = Math.sqrt(getX() * getX() + getY() * getY() + getZ() * getZ());
    // acos(s/||q||) / ||v||
    var axial_scalar = Math.acos(getW() / norm) / axial_magnitude;

    // ln(||q||) + v / ||v|| * acos(s / ||q||)
    return new Quaternion(
        scalar, 
        getX() * axial_scalar, 
        getY() * axial_scalar, 
        getZ() * axial_scalar);
  }

  /**
   * Returns W component of the quaternion.
   *
   * @return W component of the quaternion.
   */
  @JsonProperty(value = "W")
  public double getW() {
    return m_w;
  }

  /**
   * Returns X component of the quaternion.
   *
   * @return X component of the quaternion.
   */
  @JsonProperty(value = "X")
  public double getX() {
    return m_x;
  }

  /**
   * Returns Y component of the quaternion.
   *
   * @return Y component of the quaternion.
   */
  @JsonProperty(value = "Y")
  public double getY() {
    return m_y;
  }

  /**
   * Returns Z component of the quaternion.
   *
   * @return Z component of the quaternion.
   */
  @JsonProperty(value = "Z")
  public double getZ() {
    return m_z;
  }

  /**
   * Returns the rotation vector representation of this quaternion.
   *
   * <p>This is also the log operator of SO(3).
   *
   * @return The rotation vector representation of this quaternion.
   */
  public Vector<N3> toRotationVector() {
    // See equation (31) in "Integrating Generic Sensor Fusion Algorithms with
    // Sound State Representation through Encapsulation of Manifolds"
    //
    // https://arxiv.org/pdf/1107.1119.pdf
    double norm = Math.sqrt(getX() * getX() + getY() * getY() + getZ() * getZ());

    double coeff;
    if (norm < 1e-9) {
      coeff = 2.0 / getW() - 2.0 / 3.0 * norm * norm / (getW() * getW() * getW());
    } else {
      if (getW() < 0.0) {
        coeff = 2.0 * Math.atan2(-norm, -getW()) / norm;
      } else {
        coeff = 2.0 * Math.atan2(norm, getW()) / norm;
      }
    }

    return VecBuilder.fill(coeff * getX(), coeff * getY(), coeff * getZ());
  }

  // 
  public Quaternion pow(double t) {
    // q^t = e^(t * ln(q))
    return this.log().times(t).exp();
  }

  public Quaternion lerp(Quaternion other, double t) {
    // Lerp(q0, q1, t) = (1 - t) * q0 + t * q1
    return this.times(1 - t).plus(other.times(t));
  }

  public Quaternion slerp(Quaternion other, double t) {
    // Slerp(q0, q1, t) = (q1 * q0^-1) ^ t * q0
    return other.times(this.inverse()).pow(t).times(this);
  }

  @Override
  public Quaternion interpolate(Quaternion endValue, double t) {
    return slerp(endValue, t);
  }

  /**
   * Returns a uniformly random unit quaternion.
   * 
   * <p> source: https://en.wikipedia.org/wiki/3D_rotation_group#Uniform_random_sampling <p>
   * 
   * @param rand the Random Number Generator.
   * @return a uniformly sampled unit quaternion.
   */
  public static Quaternion randomUnit(Random rand) {
    var u1 = rand.nextDouble();
    var u2 = rand.nextDouble();
    var u3 = rand.nextDouble();

    var w = Math.sqrt(1 - u1) * Math.sin(2 * Math.PI * u2);
    var x = Math.sqrt(1 - u1) * Math.cos(2 * Math.PI * u2);
    var y = Math.sqrt(u1) * Math.sin(2 * Math.PI * u3);
    var z = Math.sqrt(u1) * Math.cos(2 * Math.PI * u3);

    return new Quaternion(w, x, y, z);
  }
}
