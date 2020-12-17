//
// Created by lorenzo on 01/11/20.
//

#include <exception>
struct ConnectionErrorException : public std::exception{
    const char * what () const throw () {
        return "BlueCoin connection error";
    }
};

struct SetStatusException: public std::exception{
    const char * what () const throw () {
        return "Set Status Error";
    }
};

struct GetStatusException: public std::exception{
    const char * what () const throw () {
        return "Get Status Error";
    }
};