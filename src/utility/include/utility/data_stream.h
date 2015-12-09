/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef DATA_STREAM_H_INCLUDED
#define DATA_STREAM_H_INCLUDED

#include <sstream>

#include <boost/signals2/signal.hpp>
#include <boost/function.hpp>

#include <boost/thread.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/shared_mutex.hpp>

#include <boost/date_time/posix_time/posix_time.hpp>

namespace cauv {

    class DataStreamBase {

        public:
        DataStreamBase(const std::string& name, const std::string& units, DataStreamBase* parent = NULL): m_parent(parent), m_name(name), m_units(units) {
            }

            virtual ~DataStreamBase(){}

            virtual const std::string getName(bool full=true) const {
                std::stringstream stream;

                if (full && m_parent != NULL)
                    stream << m_parent->getName() << " " << m_name;
                else stream << m_name;

                return stream.str();
            }

            virtual const std::string getUnits() const {
                return m_units;
            }

            virtual bool isMutable(){return false;}

        protected:

            const DataStreamBase* m_parent;
            const std::string m_name;
            const std::string m_units;
    };


    /** A data stream represents one type of data being sent from a data source
    *
    * @author Andy Pritchard
    */
    template <class T>

    class DataStream : public DataStreamBase {

        public:
        typedef boost::signals2::signal<void (const T&, const boost::posix_time::ptime)> signal_type;
        signal_type onUpdate;

            T m_latest;

            DataStream(const std::string& name, const std::string& units, DataStreamBase* parent = NULL):DataStreamBase(name, units, parent), m_latest(T()) {};
            DataStream(const std::string& name, DataStreamBase* parent = NULL):DataStreamBase(name, "", parent), m_latest(T()) {};

            virtual void update(const T &data) {
                // only one thread can perform any kind of updating operation at once
                // on a DataStream object, a global lock is used to enforce this
                boost::unique_lock<boost::recursive_mutex> lock(this->m_lock);
                {
                    // get a unique lock while the update takes place
                    // go back to shared lock for the onUpdate signals
                    boost::unique_lock<boost::shared_mutex> assignmentLock(m_assignmentLock);
                    this->m_latest = data;
                }
                this->onUpdate(data, boost::posix_time::microsec_clock::local_time());
            }

            template <class S>
                void update(boost::function<T(S)> &getter, S input) {
                    update(getter(input));
            }

            virtual T latest() {
                // take out a shared lock so we don't get a copy that's half way through
                // an assignment in update() in another thread
                boost::shared_lock<boost::shared_mutex> assignmentLock(m_assignmentLock);
                return this->m_latest;
            }

        protected:
            boost::recursive_mutex m_lock;
            boost::shared_mutex m_assignmentLock;
    };


    /** A data stream represents one type of data being sent from a data source that can be changed
    *
    * @author Andy Pritchard
    */
    template <class T>

    class MutableDataStream : public DataStream<T> {

        public:
            boost::signals2::signal<void(const T&)> onSet;

            MutableDataStream(const std::string& name, const std::string& units, DataStreamBase* parent = NULL):DataStream<T>(name, units, parent) {};
            MutableDataStream(const std::string& name, DataStreamBase* parent = NULL):DataStream<T>(name, "", parent) {};

            virtual bool isMutable(){return true;}

            virtual void set(const T &data) {
                boost::unique_lock<boost::recursive_mutex> lock(this->m_lock);
                // update before set as some set slots may use latest()
                this->update(data);
                this->onSet(data);
            }
    };

}

#endif // DATA_STREAMS_H_INCLUDED
