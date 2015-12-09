/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_QT_STREAMOPS_H__
#define __CAUV_QT_STREAMOPS_H__

#include <QPointF>
#include <QSizeF>
#include <QRectF>
#include <QPair>

#include <ostream>

template<typename char_T, typename traits>
std::basic_ostream<char_T, traits>& operator<<(
    std::basic_ostream<char_T, traits>& os, QPointF const& p){
    return os << "(" << p.x() << "," << p.y() << ")";
}

template<typename char_T, typename traits>
std::basic_ostream<char_T, traits>& operator<<(
    std::basic_ostream<char_T, traits>& os, QSizeF const& s){
    return os << "(" << s.width() << "x" << s.height() << ")";
}

template<typename char_T, typename traits>
std::basic_ostream<char_T, traits>& operator<<(
    std::basic_ostream<char_T, traits>& os, QRectF const& s){
    return os <<"("<< s.x() <<","<< s.y() <<":"<< s.width() <<"x"<< s.height() <<")";
}

template<typename char_T, typename traits, typename T1, typename T2>
std::basic_ostream<char_T, traits>& operator<<(
    std::basic_ostream<char_T, traits>& os, QPair<T1, T2> const& p){
    return os << "(" << p.first << "," << p.second << ")";
}


#endif //ndef __CAUV_QT_STREAMOPS_H__
