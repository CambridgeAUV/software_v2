/* Copyright 2012-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include <boost/date_time.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/thread.hpp>

namespace cauv{

class ThroughputCounter{
    private:
        typedef boost::recursive_mutex mutex_t;
        typedef boost::unique_lock<mutex_t> lock_t;

    public:
        ThroughputCounter(float const& filter_alpha=0.3)
            : m_filter_alpha(filter_alpha),
              m_last_end_time(_now()),
              m_time_taken(0),
              m_time_ratio(0),
              m_data_rate(0),
              m_frequency(0),
              m_mux(){
        }
        
        void start() {
            lock_t l(m_mux);
            m_last_start_time = _now();
        }
        void success(float num_bits) {
            end(num_bits); 
        }
        void fail() {
            end(0);
        }

        void end(float num_bits){
            boost::posix_time::ptime now = _now();
            float time_between = 0;
            if(now != m_last_end_time){
                const boost::posix_time::time_duration diff = now - m_last_end_time;
                time_between = diff.total_seconds() + diff.fractional_seconds() / 1e6;
            }else{
                // assume just twice the update frequency that we can measure:
                // if you actually need reliable counting for things occurring a
                // million times a second, get back to me...
                time_between = 2e6;
            }
                
            const boost::posix_time::time_duration start_diff = now - m_last_start_time;
            const float time_taken = start_diff.total_seconds() + start_diff.fractional_seconds() / 1e6;

            m_frequency = (1 - m_filter_alpha) * m_frequency +
                               m_filter_alpha  * 1/time_between;
            m_time_taken = (1 - m_filter_alpha) * m_time_taken +
                                m_filter_alpha  * time_taken;
            m_time_ratio = m_time_taken * m_frequency;
            
            m_data_rate = (1 - m_filter_alpha) * m_data_rate +
                               m_filter_alpha  * num_bits/time_between;
            m_last_end_time = now;
        }

        float mBitPerSecond() const{
            return m_data_rate / 1e6;
        }
        
        // in Hz
        float frequency() const{
            return m_frequency;
        }
        
        // in ms
        float time_taken() const{
            return m_time_taken * 1e3;
        }
        
        float time_ratio() const{
            return m_time_ratio;
        }

    private:
        static boost::posix_time::ptime _now(){
            return boost::posix_time::microsec_clock::universal_time();
        }
        
        // exponentially filter each datapoint with m_value = alpha*data + (1-alpha)*m_value
        const float m_filter_alpha;

        boost::posix_time::ptime m_last_start_time;
        boost::posix_time::ptime m_last_end_time;
        float m_time_taken;
        float m_time_ratio;
        float m_data_rate;
        float m_frequency;

        mutable mutex_t m_mux;
};


} // namespace cauv
