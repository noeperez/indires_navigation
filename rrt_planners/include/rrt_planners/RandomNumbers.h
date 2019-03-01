#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <cassert>

    /** \brief Random number generation. An instance of this class
        cannot be used by multiple threads at once (member functions
        are not const). However, the constructor is thread safe and
        different instances can be used safely in any number of
        threads. It is also guaranteed that all created instances will
        have a different random seed. */
    class RNG
    {
    public:

        /** \brief Constructor. Always sets a different random seed */
        RNG();

        /** \brief Generate a random real between 0 and 1 */
        double uniform01()
        {
            return uni_();
        }

        /** \brief Generate a random real within given bounds: [\e lower_bound, \e upper_bound) */
        double uniformReal(double lower_bound, double upper_bound)
        {
            assert(lower_bound <= upper_bound);
            return (upper_bound - lower_bound) * uni_() + lower_bound;
        }

        /** \brief Generate a random integer within given bounds: [\e lower_bound, \e upper_bound] */
        int uniformInt(int lower_bound, int upper_bound)
        {
            int r = (int)floor(uniformReal((double)lower_bound, (double)(upper_bound) + 1.0));
            return (r > upper_bound) ? upper_bound : r;
        }

        /** \brief Generate a random boolean */
        bool uniformBool()
        {
            return uni_() <= 0.5;
        }

        /** \brief Generate a random real using a normal distribution with mean 0 and variance 1 */
        double gaussian01()
        {
            return normal_();
        }

        /** \brief Generate a random real using a normal distribution with given mean and variance */
        double gaussian(double mean, double stddev)
        {
            return normal_() * stddev + mean;
        }

        /** \brief Generate a random real using a half-normal distribution. The value is within specified bounds [\e
            r_min, \e r_max], but with a bias towards \e r_max. The function is implemended using a Gaussian distribution with
            mean at \e r_max - \e r_min. The distribution is 'folded' around \e r_max axis towards \e r_min.
            The variance of the distribution is (\e r_max - \e r_min) / \e focus. The higher the focus,
            the more probable it is that generated numbers are close to \e r_max. */
        double halfNormalReal(double r_min, double r_max, double focus = 3.0);

        /** \brief Generate a random integer using a half-normal
            distribution. The value is within specified bounds ([\e r_min, \e r_max]), but
            with a bias towards \e r_max. The function is implemented on top of halfNormalReal() */
        int    halfNormalInt(int r_min, int r_max, double focus = 3.0);

        /** \brief Uniform random unit quaternion sampling. The computed value has the order (x,y,z,w) */
        void   quaternion(double value[4]);

        /** \brief Uniform random sampling of Euler roll-pitch-yaw angles, each in the range (-pi, pi]. The computed value has the order (roll, pitch, yaw) */
        void   eulerRPY(double value[3]);

        /** \brief Set the seed for random number generation. Use this
            function to ensure the same sequence of random numbers is
            generated. */
        static void setSeed(boost::uint32_t seed);

        /** \brief Get the seed used for random number
            generation. Passing the returned value to setSeed() at a
            subsequent execution of the code will ensure deterministic
            (repeatable) behaviour. Useful for debugging. */
        static boost::uint32_t getSeed();

    private:

        boost::mt19937                                                           generator_;
        boost::uniform_real<>                                                    uniDist_;
        boost::normal_distribution<>                                             normalDist_;
        boost::variate_generator<boost::mt19937&, boost::uniform_real<> >        uni_;
        boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > normal_;

    };

