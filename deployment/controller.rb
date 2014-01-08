# ruby >= 1.9.1

require 'socket'
require 'open3'

is_running = true

buffer = []
s = UDPSocket.new
s.bind("0.0.0.0", 22223)

stdin, stdout, stderr, wait_thr = Open3.popen3('sudo bash run.sh deployer_oodl.lua')
pid = wait_thr[:pid]

t_out = Thread.new(stdout) do
	stdout.each_line { |line| puts "[DEPL OUT:] " + line }
end

t_err = Thread.new(stderr) do
	stderr.each_line { |line| puts "[DEPL ERR:] " + line }
end

while is_running do
	text, sender = s.recvfrom(1024)

	if(text.chomp == "exit_cmd")
		is_running = false
	else
		stdin.puts text
	end
end

stdin.close
stdout.close
stderr.close

t_out.join
t_err.join
exit_status = wait_thr.value
