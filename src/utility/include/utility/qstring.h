/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_QT_STRING_H__
#define __CAUV_QT_STRING_H__

#include <QTextStream>
#include <QString>

namespace cauv{

// somewhat simpler than the std::string version
class MakeQString{
    public:
        MakeQString()
            : m_string(),
              m_stream(&m_string){
        }

        template<typename T>
        MakeQString& operator<<(T const& t){
            m_stream << t;
            return *this;
        }

        operator QString() const{
            return m_string;
        }

    private:
        QString m_string;
        QTextStream m_stream;
};

typedef MakeQString mkQStr;

} // namespace cauv

#endif // ndef __CAUV_QT_STRING_H__

