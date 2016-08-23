#ifndef OPTIONAL_PARAMETER_H
#define OPTIONAL_PARAMETER_H

template <typename T>
class OptionalParameter
{
public:
    OptionalParameter() :
        m_IsSet(false)
    {
    }

    OptionalParameter(const T &value) :
        m_Value(value),
        m_IsSet(true)
    {

    }

    OptionalParameter& operator =(const T &value)
    {
        m_Value = value;
        m_IsSet = true;

        return *this;
    }

    bool IsSet() const
    {
        return m_IsSet;
    }

    T Value() const
    {
        return m_Value;
    }

private:

    T m_Value;
    bool m_IsSet;
};

#endif // OPTIONAL_PARAMETER_H
