#ifndef FSM_H
#define FSM_H

#include <stdio.h>
#include <iostream>
#include <unordered_map>
#include <functional>
#include <vector>

template <typename States, typename ...Alphabet>
class FSM
{
private:

    States m_CurrState;
    std::unordered_map<int, std::unordered_map<int, std::function<bool(Alphabet...)>>> m_TransitionCritera;
    std::unordered_map<int, std::unordered_map<int, std::function<void(Alphabet...)>>> m_StateTransitionFunctions;



public:

    FSM ()
    {
    }

    void AddTransition(States stateFrom, States stateTo, std::function<bool(Alphabet...)> transitionCritera, std::function<void(Alphabet...)> action)
    {
        if(m_TransitionCritera.find((int)stateFrom) == m_TransitionCritera.cend())
        {
            m_TransitionCritera.insert({(int)stateFrom, {}});
            m_StateTransitionFunctions.insert({(int)stateFrom, {}});
        }

        if(m_TransitionCritera.at((int)stateFrom).find((int)stateTo) != m_TransitionCritera.at((int)stateFrom).cend())
        {
            throw std::runtime_error("Error setting up FSM: State transition already defined");
        }

        m_TransitionCritera.at((int)stateFrom).insert({(int)stateTo, transitionCritera});
        m_StateTransitionFunctions.at((int)stateFrom).insert({(int)stateTo, action});
    }

    void Transition(Alphabet... values)
    {
        States currState = m_CurrState;
        for(auto it = m_TransitionCritera.at((int)this->m_CurrState).cbegin() ; it != m_TransitionCritera.at((int)this->m_CurrState).cend() ; ++it)
        {
            States stateCheckingTransitionTo = (States)it->first;
            if(it->second(values...) == true)
            {
                m_CurrState = stateCheckingTransitionTo;
                m_StateTransitionFunctions.at((int)currState).at((int)stateCheckingTransitionTo)(values...);

                return;
            }
        }

        printf("No state change found\n");
    }
};

#endif // FSM_H
