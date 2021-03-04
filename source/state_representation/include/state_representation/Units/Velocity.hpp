#ifndef STATEREPRESENTATION_UNITS_VELOCITY_H_
#define STATEREPRESENTATION_UNITS_VELOCITY_H_

#include <chrono>
#include "state_representation/Units/Distance.hpp"
#include "state_representation/Units/Angle.hpp"

using namespace std::chrono_literals;

namespace state_representation
{
	namespace Units
	{
		template <class T>
		class Velocity;

		using LinearVelocity = Velocity<Distance>;
		using AngularVelocity = Velocity<Angle>;

		inline namespace literals
		{
			/**
			 * @brief Literal operator to create a Velocity in meter / second
			 * @param n the Velocity value in meter / second
			 * @param the Velocity in the base unit (meter / second)
			 */
			constexpr LinearVelocity operator""_m_s(long double n);

			/**
			 * @brief Literal operator to create a Velocity in meter / hour
			 * @param n the Velocity value in meter / hour
			 * @param the Velocity in the base unit (meter / second)
			 */
			constexpr LinearVelocity operator""_m_h(long double n);

			/**
			 * @brief Literal operator to create a Velocity in meter / millisecond
			 * @param n the Velocity value in meter / millisecond
			 * @param the Velocity in the base unit (meter / second)
			 */
			constexpr LinearVelocity operator""_m_ms(long double n);

			/**
			 * @brief Literal operator to create a Velocity in kilometer / hour
			 * @param n the Velocity value in kilometer / hour
			 * @param the Velocity in the base unit (meter / second)
			 */
			constexpr LinearVelocity operator""_km_h(long double n);

			/**
			 * @brief Literal operator to create a Velocity in kilometer / second
			 * @param n the Velocity value in kilometer / second
			 * @param the Velocity in the base unit (meter / second)
			 */
			constexpr LinearVelocity operator""_km_s(long double n);

			/**
			 * @brief Literal operator to create a Velocity in kilometer / millisecond
			 * @param n the Velocity value in kilometer / millisecond
			 * @param the Velocity in the base unit (meter / second)
			 */
			constexpr LinearVelocity operator""_km_ms(long double n);

			/**
			 * @brief Literal operator to create a Velocity in millimeter / hour
			 * @param n the Velocity value in millimeter / hour
			 * @param the Velocity in the base unit (meter / second)
			 */
			constexpr LinearVelocity operator""_mm_h(long double n);

			/**
			 * @brief Literal operator to create an AngularVelocity in radian / second
			 * @param n the Velocity value in radian / second
			 * @param the Velocity in the base unit (radian / second)
			 */
			constexpr AngularVelocity operator""_rad_s(long double n);

			/**
			 * @brief Literal operator to create an AngularVelocity in degree / second
			 * @param n the Velocity value in degree / second
			 * @param the Velocity in the base unit (radian / second)
			 */
			constexpr AngularVelocity operator""_deg_s(long double n);
		}

		template <class T>
		class Velocity
		{
		private:
			long double value; ///< value of the velocity in the base unit of T

		public:
			/**
			 * @brief Constructor with a value in the base unit of T
			 * @param n the value in the base unit of T
			 */
			constexpr Velocity(long double n=0.0);

			/**
			 * @brief Copy constructor from another Velocity
			 * @param vel the Velocity to copy
			 */
			constexpr Velocity(const Velocity<T>& vel);

			/**
			 * @brief Getter of the value attribute
			 * @return the value in meter per second
			 */
			constexpr long double get_value() const;

			/**
			 * @brief Overload the - operator
			 * @return the negative velocity in the base unit
			 */
			constexpr Velocity<T>& operator-();

			/**
		 	 * @brief Overload the += operator
		 	 * @param rhs Velocity to add
		 	 * @return the current Velocity added the Velocity given in argument
		     */
			constexpr Velocity<T>& operator+=(const Velocity<T>& rhs);

			/**
		 	 * @brief Overload the + operator
		 	 * @param rhs Velocity to add
		 	 * @return the current Velocity added the Velocity given in argument
		     */
			constexpr Velocity<T> operator+(const Velocity<T>& rhs) const;

			/**
		 	 * @brief Overload the -= operator
		 	 * @param rhs Velocity to substract
		 	 * @return the current Velocity minus the Velocity given in argument
		     */
			constexpr Velocity<T>& operator-=(const Velocity<T>& rhs);

			/**
		 	 * @brief Overload the - operator
		 	 * @param rhs Velocity to substract
		 	 * @return the current Velocity minus the Velocity given in argument
		     */
			constexpr Velocity<T> operator-(const Velocity<T>& rhs) const;

			/**
		 	 * @brief Overload the *= operator with a scalar
		 	 * @param lambda the scalar to multiply with
		 	 * @return the Velocity multiply by lambda
		     */
			constexpr Velocity<T>& operator*=(double lambda);

			/**
		 	 * @brief Overload the * operator with a scalar
		 	 * @param lambda the scalar to multiply with
		 	 * @return the Velocity multiply by lambda
		     */
			constexpr Velocity<T> operator*(double lambda) const;

			/**
		 	 * @brief Overload the /= operator with a scalar
		 	 * @param lambda the scalar to divide by
		 	 * @return the Velocity divided by lambda
		     */
			constexpr Velocity<T>& operator/=(double lambda);

			/**
		 	 * @brief Overload the / operator with a scalar
		 	 * @param lambda the scalar to divide by
		 	 * @return the Velocity divided by lambda
		     */
			constexpr Velocity<T> operator/(double lambda) const;

			/**
			 * @brief Overload the == operator
			 * @param rhs the other Velocity to check equality with
			 * @return bool true if the two Velocitys are equal
			 */
			constexpr bool operator==(const Velocity<T>& rhs) const;

			/**
			 * @brief Overload the != operator
			 * @param rhs the other Velocity to check inequality with
			 * @return bool true if the two Velocitys are different
			 */
			constexpr bool operator!=(const Velocity<T>& rhs) const;

			/**
			 * @brief Overload the > operator
			 * @param rhs the other Velocity to check strict superiority with
			 * @return bool true if the current Velocity is strictly superior than provided Velocity
			 */
			constexpr bool operator>(const Velocity<T>& rhs) const;

			/**
			 * @brief Overload the > operator
			 * @param rhs the other Velocity to check superiority with
			 * @return bool true if the current Velocity is superior than provided Velocity
			 */
			constexpr bool operator>=(const Velocity<T>& rhs) const;

			/**
			 * @brief Overload the < operator
			 * @param rhs the other Velocity to check strict inferiority with
			 * @return bool true if the current Velocity is strictly inferior than provided Velocity
			 */
			constexpr bool operator<(const Velocity<T>& rhs) const;

			/**
			 * @brief Overload the < operator
			 * @param rhs the other Velocity to check inferiority with
			 * @return bool true if the current Velocity is inferior than provided Velocity
			 */
			constexpr bool operator<=(const Velocity<T>& rhs) const;

			/**
		 	 * @brief Overload the / operator between two Velocitys
		 	 * @param lhs the first Velocity
		 	 * @param rhs the second Velocity
		 	 * @return the ratio between first and second Velocity
		     */
			friend constexpr double operator/(const Velocity<T>& lhs, const Velocity<T>& rhs)
			{
				return lhs.value / rhs.value;
			}

			/**
		 	 * @brief Overload the * operator with a scalar on the left side
		 	 * @param lambda the scalar to multiply with
		 	 * @return the Velocity multiply by lambda
		     */
			friend constexpr Velocity<T> operator*(double lambda, const Velocity<T>& rhs)
			{
				return Velocity<T>(lambda * rhs.value);
			}

			/**
		 	 * @brief Overload the / operator for a T divided by a time period
		 	 * @param lhs the T unit
		 	 * @param rhs the time period
		 	 * @return the Velocity as the T over the time period
		     */
			template <class Rep, class DurationRatio>
			friend constexpr Velocity<T> operator/(const T& lhs, const std::chrono::duration<Rep, DurationRatio>& rhs);
		};

		template <class T>
		constexpr Velocity<T>::Velocity(long double n):
		value(n)
		{}

		template <class T>
		constexpr Velocity<T>::Velocity(const Velocity<T>& vel):
		value(vel.value)
		{}

		template <class T>
		constexpr long double Velocity<T>::get_value() const
		{
			return this->value;
		}

		template <class T>
		constexpr Velocity<T>& Velocity<T>::operator-()
		{
			this->value = -this->value;
			return (*this);
		}

		template <class T>
		constexpr Velocity<T>& Velocity<T>::operator+=(const Velocity<T>& rhs)
		{
			this->value = this->value + rhs.value;
			return (*this);
		}

		template <class T>
		constexpr Velocity<T> Velocity<T>::operator+(const Velocity<T>& rhs) const
		{
			Velocity<T> result(*this);
			result += rhs;
			return result;
		}

		template <class T>
		constexpr Velocity<T>& Velocity<T>::operator-=(const Velocity<T>& rhs)
		{
			this->value = this->value - rhs.value;
			return (*this);
		}

		template <class T>
		constexpr Velocity<T> Velocity<T>::operator-(const Velocity<T>& rhs) const
		{
			Velocity<T> result(*this);
			result -= rhs;
			return result;
		}

		template <class T>
		constexpr Velocity<T>& Velocity<T>::operator*=(double lambda)
		{
			this->value = this->value * lambda;
			return (*this);
		}

		template <class T>
		constexpr Velocity<T> Velocity<T>::operator*(double lambda) const
		{
			Velocity<T> result(*this);
			result *= lambda;
			return result;
		}

		template <class T>
		constexpr Velocity<T>& Velocity<T>::operator/=(double lambda)
		{
			this->value = this->value / lambda;
			return (*this);
		}

		template <class T>
		constexpr Velocity<T> Velocity<T>::operator/(double lambda) const
		{
			Velocity<T> result(*this);
			result /= lambda;
			return result;
		}

		template <class T>
		constexpr bool Velocity<T>::operator==(const Velocity<T>& rhs) const
		{
			return (abs(this->value - rhs.value) < 1e-4);
		}

		template <class T>
		constexpr bool Velocity<T>::operator!=(const Velocity<T>& rhs) const
		{
			return !((*this) == rhs);
		}

		template <class T>
		constexpr bool Velocity<T>::operator>(const Velocity<T>& rhs) const
		{
			return ((this->value - rhs.value) > 1e-4);
		}

		template <class T>
		constexpr bool Velocity<T>::operator>=(const Velocity<T>& rhs) const
		{
			return (((*this) > rhs) or ((*this) == rhs));
		}

		template <class T>
		constexpr bool Velocity<T>::operator<(const Velocity<T>& rhs) const
		{
			return ((rhs.value - this->value) > 1e-4);
		}

		template <class T>
		constexpr bool Velocity<T>::operator<=(const Velocity<T>& rhs) const
		{
			return (((*this) < rhs) or ((*this) == rhs));
		}

		template <class T, class Rep, class DurationRatio>
		constexpr Velocity<T> operator/(const T& dist, const std::chrono::duration<Rep, DurationRatio>& rhs)
		{
			const auto rhsInSeconds = std::chrono::duration_cast<std::chrono::seconds>(rhs);
			return Velocity<T>(dist.get_value()/rhsInSeconds.count());
		}

		inline namespace literals
		{
			constexpr LinearVelocity operator""_m_s(long double n)
			{
				Distance d = 1.0_m;
				auto t = 1.0s;
				return n * (d / t);
			}

			constexpr LinearVelocity operator""_m_h(long double n)
			{
				Distance d = 1.0_m;
				auto t = 1.0h;
				return n * (d / t);
			}

			constexpr LinearVelocity operator""_m_ms(long double n)
			{
				Distance d = 1.0_m;
				auto t = 1.0ms;
				return n * (d / t);
			}

			constexpr LinearVelocity operator""_km_h(long double n)
			{
				Distance d = 1.0_km;
				auto t = 1.0h;
				return n * (d / t);
			}

			constexpr LinearVelocity operator""_km_s(long double n)
			{
				Distance d = 1.0_km;
				auto t = 1.0s;
				return n * (d / t);
			}

			constexpr LinearVelocity operator""_km_ms(long double n)
			{
				Distance d = 1.0_km;
				auto t = 1.0ms;
				return n * (d / t);
			}

			constexpr LinearVelocity operator""_mm_h(long double n)
			{
				Distance d = 1.0_mm;
				auto t = 1.0h;
				return n * (d / t);
			}

			constexpr AngularVelocity operator""_rad_s(long double n)
			{
				Angle a = 1.0_rad;
				auto t = 1.0s;
				return n * (a / t);
			}

			constexpr AngularVelocity operator""_deg_s(long double n)
			{
				Angle a = 1.0_deg;
				auto t = 1.0s;
				return n * (a / t);
			}
		}
	}
}

#endif