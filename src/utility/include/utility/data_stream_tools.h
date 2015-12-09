/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef DATA_STREAM_TOOLS_H_INCLUDED
#define DATA_STREAM_TOOLS_H_INCLUDED

#include "data_stream.h"

#include <generated/types/floatYPR.h>
#include <generated/types/floatXYZ.h>
#include <generated/types/MotorDemand.h>

#include <iostream>

#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/signals/trackable.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

namespace cauv {

    class DataStreamTool  : public boost::signals2::trackable {};

    /** A DataStreamSplitter can be used to separate streams of structured data
    *   must be extended to be used
    *
    * @author Andy Pritchard
    */
    template<class T>

    class DataStreamSplitter : public DataStreamTool {};

    /* floatYPR specialization */
    template<>

    class DataStreamSplitter<cauv::floatYPR> : public DataStreamTool {

    public:
        DataStreamSplitter<cauv::floatYPR>(boost::shared_ptr<DataStream<cauv::floatYPR> > stream) :
                combined(stream),
                yaw(boost::make_shared<DataStream<float> >("Yaw", stream->getUnits())),
                pitch(boost::make_shared<DataStream<float> >("Pitch", stream->getUnits())),
                roll(boost::make_shared<DataStream<float> >("Roll", stream->getUnits()))
        {
            // getter function binds
            boost::function<float(cauv::floatYPR)> yawGetter = boost::bind(&cauv::floatYPR::yaw, _1);
            boost::function<float(cauv::floatYPR)> pitchGetter = boost::bind(&cauv::floatYPR::pitch, _1);
            boost::function<float(cauv::floatYPR)> rollGetter = boost::bind(&cauv::floatYPR::roll, _1);

            // connect up the slots
            // the getter functions are passed to the new data streams and evaluated there
            stream->onUpdate.connect(boost::bind(&DataStream<float>::update<cauv::floatYPR>, yaw.get(), yawGetter, _1));
            stream->onUpdate.connect(boost::bind(&DataStream<float>::update<cauv::floatYPR>, pitch.get(), pitchGetter, _1));
            stream->onUpdate.connect(boost::bind(&DataStream<float>::update<cauv::floatYPR>, roll.get(), rollGetter, _1));
        }

        boost::shared_ptr<DataStream<floatYPR> > combined;
        boost::shared_ptr<DataStream<float> > yaw;
        boost::shared_ptr<DataStream<float> > pitch;
        boost::shared_ptr<DataStream<float> > roll;
    };

    /* floatXYZ specialization */
    template<>

    class DataStreamSplitter<cauv::floatXYZ> : public DataStreamTool {

    public:
        DataStreamSplitter<cauv::floatXYZ>(boost::shared_ptr<DataStream<cauv::floatXYZ> > stream) :
                combined(stream),
                x(boost::make_shared<DataStream<float> >("X", stream->getUnits())),
                y(boost::make_shared<DataStream<float> >("Y", stream->getUnits())),
                z(boost::make_shared<DataStream<float> >("Z", stream->getUnits()))
        {
            // getter function binds
            boost::function<float(cauv::floatXYZ)> xGetter = boost::bind(&cauv::floatXYZ::x, _1);
            boost::function<float(cauv::floatXYZ)> yGetter = boost::bind(&cauv::floatXYZ::y, _1);
            boost::function<float(cauv::floatXYZ)> zGetter = boost::bind(&cauv::floatXYZ::z, _1);

            // connect up the slots
            // the getter functions are passed to the new data streams and evaluated there
            stream->onUpdate.connect(boost::bind(&DataStream<float>::update<cauv::floatXYZ>, x.get(), xGetter, _1));
            stream->onUpdate.connect(boost::bind(&DataStream<float>::update<cauv::floatXYZ>, y.get(), yGetter, _1));
            stream->onUpdate.connect(boost::bind(&DataStream<float>::update<cauv::floatXYZ>, z.get(), zGetter, _1));
        }

        boost::shared_ptr<DataStream<floatXYZ> > combined;
        boost::shared_ptr<DataStream<float> > x;
        boost::shared_ptr<DataStream<float> > y;
        boost::shared_ptr<DataStream<float> > z;
    };

    /* MotorDemand specialization */
    template<>

    class DataStreamSplitter<cauv::MotorDemand> : public DataStreamTool {

    public:
        DataStreamSplitter<cauv::MotorDemand>(boost::shared_ptr<DataStream<cauv::MotorDemand> > stream) :
                combined(stream),
                prop(boost::make_shared<DataStream<float> >("prop", stream->getUnits(), stream.get())),
                hbow(boost::make_shared<DataStream<float> >("hbow", stream->getUnits(), stream.get())),
                vbow(boost::make_shared<DataStream<float> >("vbow", stream->getUnits(), stream.get())),
                hstern(boost::make_shared<DataStream<float> >("hstern", stream->getUnits(), stream.get())),
                vstern(boost::make_shared<DataStream<float> >("vstern", stream->getUnits(), stream.get()))
        {
            // getter function binds
            boost::function<float(cauv::MotorDemand)> propGetter = boost::bind(&cauv::MotorDemand::prop, _1);
            boost::function<float(cauv::MotorDemand)> hbowGetter = boost::bind(&cauv::MotorDemand::hbow, _1);
            boost::function<float(cauv::MotorDemand)> vbowGetter = boost::bind(&cauv::MotorDemand::vbow, _1);
            boost::function<float(cauv::MotorDemand)> hsternGetter = boost::bind(&cauv::MotorDemand::hstern, _1);
            boost::function<float(cauv::MotorDemand)> vsternGetter = boost::bind(&cauv::MotorDemand::vstern, _1);

            // connect up the slots
            // the getter functions are passed to the new data streams and evaluated there
            stream->onUpdate.connect(boost::bind(&DataStream<float>::update<cauv::MotorDemand>, prop.get(), propGetter, _1));
            stream->onUpdate.connect(boost::bind(&DataStream<float>::update<cauv::MotorDemand>, hbow.get(), hbowGetter, _1));
            stream->onUpdate.connect(boost::bind(&DataStream<float>::update<cauv::MotorDemand>, vbow.get(), vbowGetter, _1));
            stream->onUpdate.connect(boost::bind(&DataStream<float>::update<cauv::MotorDemand>, hstern.get(), hsternGetter, _1));
            stream->onUpdate.connect(boost::bind(&DataStream<float>::update<cauv::MotorDemand>, vstern.get(), vsternGetter, _1));
        }

        boost::shared_ptr<DataStream<MotorDemand> > combined;
        boost::shared_ptr<DataStream<float> > prop;
        boost::shared_ptr<DataStream<float> > hbow;
        boost::shared_ptr<DataStream<float> > vbow;
        boost::shared_ptr<DataStream<float> > hstern;
        boost::shared_ptr<DataStream<float> > vstern;
    };

    /** A DataStreamRecorder can be used to log a certain amount of input from a DataStream. Useful
    * for graphing the data
    *
    * @author Andy Pritchard
    */
    template<class T>

    class DataStreamRecorder : public DataStreamTool {

    public:
        DataStreamRecorder<T>(boost::shared_ptr<DataStream<T> > stream, const unsigned int samples = 100000, const unsigned int frequency = 1):
                m_numSamples(samples), m_sampleFrequency(frequency) {
            stream->onUpdate.connect(boost::bind(&DataStreamRecorder<T>::change, this, _1, _2));
        };

        void setNumSamples(const unsigned int samples){
            boost::mutex::scoped_lock lock(m_mutex);
            m_numSamples = samples;
            if(m_numSamples < m_history.size()) {
                m_timestamps.resize(m_numSamples);
                m_history.resize(m_numSamples);
            }
        }

        unsigned int getNumSamples(){
            boost::mutex::scoped_lock lock(m_mutex);
            return m_numSamples;
        }

        void setSampleFrequency(const unsigned int frequency){
            {
                boost::mutex::scoped_lock lock(m_mutex);
                m_sampleFrequency = frequency;
                m_frequencyCount = frequency; // set to frequency as ooposed to 0 so we include the next sample
            }
            this->clear();
        }

        unsigned int getSampleFrequency() {
            return m_sampleFrequency;
        }

        const std::vector<T> getHistory() const {
            boost::mutex::scoped_lock lock(m_mutex);
            return m_history;
        }

        const std::vector<boost::posix_time::ptime> getTimestamps() const {
            boost::mutex::scoped_lock lock(m_mutex);
            return m_timestamps;
        }

        void clear() {
            boost::mutex::scoped_lock lock(m_mutex);
            m_history.clear();
            m_timestamps.clear();
        }

        virtual void change(const T &data, const boost::posix_time::ptime timestamp) {
            boost::mutex::scoped_lock lock(m_mutex);

            if (!m_history.empty() && m_history.size() > m_numSamples) {
                m_history.erase(m_history.begin());
                m_timestamps.erase(m_timestamps.begin());

            }

            if(++m_frequencyCount >= m_sampleFrequency) {
                m_history.push_back(data);
                m_timestamps.push_back(timestamp);
                m_frequencyCount = 0;
            }
        };

    protected:
        boost::mutex m_mutex;
        std::vector<T> m_history;
        std::vector<boost::posix_time::ptime> m_timestamps;
        unsigned int m_numSamples;
        unsigned int m_sampleFrequency;
        unsigned int m_frequencyCount;
    };



    /** A DataStreamPrinter is used to print the data coming from a DataStream
    *
    * @author Andy Pritchard
    */
    template<class T>

    class DataStreamPrinter : public DataStreamTool {

    public:
        DataStreamPrinter<T>(boost::shared_ptr<DataStream<T> > stream, std::ostream &output):
                m_ostream(output) {
            m_connection = stream->onUpdate.connect(boost::bind(&DataStreamPrinter<T>::change, this, _1));
        };

        DataStreamPrinter<T>(boost::shared_ptr<DataStream<T> > stream):
                m_ostream(std::cout) {
            m_connection = stream->onUpdate.connect(boost::bind(&DataStreamPrinter<T>::change, this, _1));
        };

        void change(const T &data) {
            m_ostream << data;
        };

    protected:
        boost::signals::connection m_connection;
        std::ostream& m_ostream;
    };

}

#endif // DATA_STREAM_TOOLS_H_INCLUDED
