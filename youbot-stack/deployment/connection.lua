function udp_server()
    local socket = require "socket"
    local udp = socket.udp()

    udp:settimeout(0)
    udp:setsockname('localhost', 123456)

    local world = {} -- the empty world-state

    local data, msg_or_ip, port_or_nil
    local entity, cmd, parms

    local running = true

    print "Beginning server loop."
    while running do
        data, msg_or_ip, port_or_nil = udp:receivefrom()
        if data then

            --manipulate data

        elseif msg_or_ip ~= 'timeout' then
            error("Unknown network error: "..tostring(msg))
        end
        --socket.sleep(0.01)
        --coroutine.yield()
    end
end