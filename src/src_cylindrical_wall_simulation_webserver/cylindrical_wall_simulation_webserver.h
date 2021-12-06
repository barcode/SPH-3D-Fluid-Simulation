#pragma once
#include <tuple>
#include <type_traits>
#include <atomic>
#include <cstdint>

template <typename T>
class triple_buffer
{
public:
    template<class U = T>
    triple_buffer(std::enable_if_t<std::is_default_constructible_v<U>>* = nullptr)
    {}

    template<class U = T>
    triple_buffer(const T& init, std::enable_if_t<std::is_copy_constructible_v<U>>* = nullptr) :
        buffer{init, init, init}
    {}

    template<class U = T>
    triple_buffer(T&& init_read, T&& init_write, T&& init_hidden, std::enable_if_t<std::is_copy_constructible_v<U>>* = nullptr) :
        buffer{std::move(init_read), std::move(init_hidden), std::move(init_write)}
    {
        static_assert (read_index_shift < hidden_index_shift);
        static_assert (hidden_index_shift < write_index_shift);
    }

    // non-copyable behavior
    triple_buffer<T>(const triple_buffer<T>&) = delete;
    triple_buffer<T>& operator=(const triple_buffer<T>&) = delete;

    /// @return the write buffer
    T& getWriteBuffer()
    {
        return buffer[getWriteBufferIndex()];
    }
    /// @return the write buffer
    const T& getWriteBuffer() const
    {
        return buffer[getWriteBufferIndex()];
    }
    /// @return the current read buffer
    const T& getReadBuffer() const
    {
        return buffer[getReadBufferIndex() ];
    }
    /// @return the most up to date read buffer
    const T& getUpToDateReadBuffer() const
    {
        updateReadBuffer();
        return getReadBuffer();
    }

    /**
     * @brief Swaps in the hidden buffer if it contains new data.
     * @return True if new data is available
     */
    bool updateReadBuffer() const
    {
        std::uint_fast8_t flagsNow(flags.load(std::memory_order_consume));

        if (!hasNewWrite(flagsNow))
        {
            return false;
        }

        while (!flags.compare_exchange_weak(flagsNow, swap_read_hidden(flagsNow), std::memory_order_release, std::memory_order_consume));
        return true;
    }

    void commitWrite()
    {
        swapWriteAndHiddenBuffer();
    }

    template<class U = T>
    typename std::enable_if<std::is_copy_constructible<U>::value>::type reinitAllBuffers(const T& init)
    {
        buffer[0] = init;
        buffer[1] = init;
        buffer[2] = init;
    }
    void reinitAllBuffers(T&& writeBuff, T&& hiddenBuff, T&& readBuff)
    {
        buffer[getWriteBufferIndex() ] = std::move(writeBuff);
        buffer[getHiddenBufferIndex()] = std::move(hiddenBuff);
        buffer[getReadBufferIndex()  ] = std::move(readBuff);
    }

protected:

    // /////////////////////////////////////////////////////////////////////////////// //
    // get indices
    std::uint_fast8_t getReadBufferIndex()   const
    {
        return (flags.load(std::memory_order_consume) & read_index_mask) >> read_index_shift ;
    }
    std::uint_fast8_t getWriteBufferIndex()  const
    {
        return (flags.load(std::memory_order_consume) & write_index_mask) >> write_index_shift;
    }
    std::uint_fast8_t getHiddenBufferIndex() const
    {
        return (flags.load(std::memory_order_consume) & hidden_index_mask) >> hidden_index_shift;
    }

    // /////////////////////////////////////////////////////////////////////////////// //
    //buffer operations
    /// @brief Swap the write buffer with the hidden buffer
    void swapWriteAndHiddenBuffer()
    {
        std::uint_fast8_t flagsNow(flags.load(std::memory_order_consume));
        while (!flags.compare_exchange_weak(flagsNow, swap_write_hidden(flagsNow), std::memory_order_release, std::memory_order_consume));
    }
    // void swapReadAndHiddenBuffer(); is done by updateReadBuffer()

    // /////////////////////////////////////////////////////////////////////////////// //
    //flag opertions
    // check if the newWrite bit is 1
    bool hasNewWrite(uint_fast8_t flags) const
    {
        return flags & dirty_bit_mask;
    }
    /// swap read and hidden indexes
    /**
     * @brief swap read and hidden indexes of given flags (set dirty to 0)
     * @param flags the current flags
     * @return the new flags
     */
    static constexpr std::uint_fast8_t swap_read_hidden(uint_fast8_t flags);

    /**
     * @brief swap write and hidden indexes of given flags (set dirty to 1)
     * @param flags the current flags
     * @return the new flags
     */
    static constexpr std::uint_fast8_t swap_write_hidden(uint_fast8_t flags);

    // /////////////////////////////////////////////////////////////////////////////// //
    // constants
    static constexpr std::uint_fast8_t read_index_shift = 0;
    static constexpr std::uint_fast8_t hidden_index_shift = 2;
    static constexpr std::uint_fast8_t write_index_shift = 4;

    static constexpr std::uint_fast8_t hidden_read_shift  = hidden_index_shift - read_index_shift;
    static constexpr std::uint_fast8_t hidden_write_shift = write_index_shift - hidden_index_shift;

    static constexpr std::uint_fast8_t read_index_mask   = 0b00'00'00'11 << read_index_shift  ; //0b00'00'00'11;
    static constexpr std::uint_fast8_t hidden_index_mask = 0b00'00'00'11 << hidden_index_shift; //0b00'00'11'00;
    static constexpr std::uint_fast8_t write_index_mask  = 0b00'00'00'11 << write_index_shift ; //0b00'11'00'00;
    static constexpr std::uint_fast8_t dirty_bit_mask    = 0b10'00'00'00;
    // initially dirty = 0, write 0, hidden = 1, read = 2
    static const std::uint_fast8_t initial_flags          = 0b00'00'01'10; //= 2 + (1 << 2); //0b00000110; w=0, h=1, r=0

    // /////////////////////////////////////////////////////////////////////////////// //
    // data
    mutable std::atomic_uint_fast8_t flags {initial_flags};
    std::array<T, 3> buffer;
};

template<typename T>
inline constexpr std::uint_fast8_t triple_buffer<T>::swap_read_hidden(std::uint_fast8_t flags)
{
    return 0      //set dirty bit to 0
            | ((flags & hidden_index_mask) >> hidden_read_shift) // hidden index now is read   index
            | ((flags & read_index_mask) << hidden_read_shift)     // read   index now is hidden index
            | (flags & write_index_mask);   // keep write index
}

template<typename T>
inline constexpr std::uint_fast8_t triple_buffer<T>::swap_write_hidden(std::uint_fast8_t flags)
{
    return dirty_bit_mask         //set dirty bit to 1
           | ((flags & hidden_index_mask) << hidden_write_shift) // hidden index now is write  index
           | ((flags & write_index_mask) >> hidden_write_shift)  // write  index now is hidden index
           | (flags & read_index_mask);    // keep read index
}


///////////////////////////////////////////////////////////////////
#include <thread>
#include <atomic>
#include <mutex>

#include "cylindrical_wall_simulation.h"

class cylindrical_wall_simulation_webserver
{
public:
    struct simulator_state
    {
        static std::int64_t now()
        {
            return std::chrono::high_resolution_clock::now().time_since_epoch().count();
        }
        std::atomic<double> setup_radius_inner    {0.15};
        std::atomic<double> setup_radius_outer    {0.18};
        std::atomic<double> setup_height          {1};
        std::atomic_size_t  setup_num_particles   {2500};
        std::atomic<double> setup_radius_particles{0.03};
        std::atomic<bool>   setup_reset_sim       {false};

        std::atomic<bool>   sim_zero_vel          {false};
        std::atomic<bool>   sim_catch_particles   {true};
        std::atomic<double> sim_gravity_x         {0};
        std::atomic<double> sim_gravity_y         {-3};
        std::atomic<double> sim_gravity_z         {0};
        std::atomic<double> sim_dt_per_step       {0.01};
        std::atomic<double> sim_GAS_STIFFNESS     {3};
        std::atomic<double> sim_REST_DENSITY      {900};
        std::atomic<double> sim_PARTICLE_MASS     {0.02};
        std::atomic<double> sim_VISCOSITY         {3.5};
        std::atomic<double> sim_SURFACE_THRESHOLD {7};
        std::atomic<double> sim_SURFACE_TENSION   {1};
        std::atomic<double> sim_KERNEL_PARTICLES  {20};
        std::atomic<double> sim_WALL_K            {10000};
        std::atomic<double> sim_WALL_DAMPING      {-1};

        std::atomic<double> brightness            {10000};
        std::atomic_size_t  num_diodes_height     {100};
        std::atomic_size_t  num_diodes_width      {100};
        std::atomic_size_t  light_radius_height   {0};
        std::atomic_size_t  light_radius_width    {0};
        std::atomic<double> d_brigh_to_pxval      {100};
    };
    struct sim_output
    {
        std::vector<VEC3D>  centers;
        struct out_lights
        {
            std::vector<double> lights;
            std::size_t         lights_w = 0;
            std::size_t         lights_h = 0;
        };
        out_lights lights;
    };

    cylindrical_wall_simulation_webserver(const std::string &ip,
            unsigned short port);
    ~cylindrical_wall_simulation_webserver();
    cylindrical_wall_simulation_webserver(cylindrical_wall_simulation_webserver&&)=delete;
    cylindrical_wall_simulation_webserver(const cylindrical_wall_simulation_webserver&)=delete;
    cylindrical_wall_simulation_webserver& operator=(cylindrical_wall_simulation_webserver&&)=delete;
    cylindrical_wall_simulation_webserver& operator=(const cylindrical_wall_simulation_webserver&)=delete;

    sim_output::out_lights lights() const;
    std::vector<VEC3D> centers() const;

    void wait();

private:
    void task_sim();
    void task_web();

    std::string ip;
    unsigned short port;

    simulator_state sim_state;
    triple_buffer<sim_output> output;
    mutable std::mutex output_read_mutex;
    std::thread thread_sim;
    std::thread thread_web;
    std::atomic_bool stop_thread_sim{false};
    std::atomic_bool stop_thread_web{false};
};
