#include "Motor_model.h"

template <typename T>
Motor_model<T>::Motor_model()
{
}

template <typename T>
Motor_model<T>::~Motor_model()
{
}

template <typename T>
void Motor_model<T>::assign_data(T (*data_wv_)[2], uint8_t num_data_)
{
    data_wv = data_wv_;
    num_data = num_data_;
}

template <typename T>
T Motor_model<T>::getV(T w)
{
    if (w <= data_wv[0][0])
    {
        return data_wv[0][1];
    }
    else if (w >= data_wv[num_data - 1][0])
    {
        return data_wv[num_data - 1][1];
    }
    else
    {
        uint8_t index_1 = 0;
        uint8_t index_2 = 0;

        for (int i = 0; i < num_data; i++)
        {
            if (w >= data_wv[i][0])
            {
                index_1 = i;
            }
            else
            {
                break;
            }
        }

        index_2 = index_1 + 1;

        if (index_2 > (num_data - 1))
        {
            index_2 = num_data - 1;
        }

        if ((data_wv[index_1][0] == 0) || (data_wv[index_2][0] == 0))
        {
            if (w <= data_wv[index_1][0])
            {
                return data_wv[index_1][0];
            }
            else if (w >= data_wv[index_2][0])
            {
                return data_wv[index_2][0];
            }
            else
            {
                return T(0);
            }
        }
        else
        {
            return data_wv[index_1][1] + (w - data_wv[index_1][0]) * (data_wv[index_2][1] - data_wv[index_1][1]) / (data_wv[index_2][0] - data_wv[index_1][0]);
        }
    }

    return T(0);
}