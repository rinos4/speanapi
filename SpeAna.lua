#!/usr/bin/luajit
-- -----------------------------------------------------------------------------
-- Spectrum analyzer for RaspberryPi version 0.1 by rinos4u
-- -----------------------------------------------------------------------------
-- 2013.04.20 rinos4u	new

package.path = package.path .. ";LJIT2RPi/?.lua"

-- -----------------------------------------------------------------------------
-- Defines for option
-- -----------------------------------------------------------------------------
local USAGE = [[
 Usage: SpeAna.lua [-c|-g] [-fb size] [-fc ch] [-fo ration] [-fw winfn]
                   [-p size] [-ms sr] [-md dur] [-m dev | -s | wav_file...]
                   [-r n] [-h]

  [Display mode]
	-c: CUI mode 				(default: false)
	-g: GUI(DMX) mode			(default: true, required HDMI display)

  [FFT parameter]
	-fb: Set FFT block size      (default: 2048, power syntax is OK, e.g. 2^10)
	-fc: Set FFT target channel  (default: 0 [merged])
	-fo: Set FFT overlap ratio   (default: 0.5, fraction syntax is OK, e.g. 3/4)
	-fw: Set FFT window function (default: Hann)
         (Available windows functions: Rect, Hann, Hamming, Bartlett, Blackman, Sine)

  [Input source] 
	wav_file: Use wave files instead of mic.
	-p size : Set pre-laod threshold size [bytes] for file input (default: 10MB)

	-s      : Use stdin instead of mic/file (It also ignores -l option)

	-m  dev : Set dev for mic input device (default: "sysdefault:Dongle")
	-ms sr  : Set mic sampling ratio (default: 48000 [Hz])
	-md dur : Set mic input duration (default: 3600  [s])

  [Others] 
	-r n: Repeat specified time for FFT  (default: 1)
	-h  : Show this help
]]

-- Default values of option
local options = {
	files		= {},					-- If file is specified use it instead of mic input
	dmx			= true,					-- DisplayManX mode (Set false for CUI mode)

	fft_block	= 2048,					-- 2KB block
	fft_ch    	= 0,					-- 0 is merge of all source channel
	fft_overlap	= 1/2,					-- 50%
	fft_window	= "HANN",				-- Hann window

	preload		= 10 * 1024 * 1024,		-- 1MB
	stdin		= false,				-- If stdin is specified use it instead of mic/file
	mic			= "sysdefault:Dongle",	-- Use default USB Sound Dongle (ALSA)
	mic_sr		= 48000,				-- 48kHz
	mic_dur		= 3600,					-- 1H
	mic_bit		= "S16_LE",				-- 16bit sampling

	loop		= 1,					-- Repeat count for FFT

	min_dB		= -120,					-- Use decibel mode (Set false to use linear mode)
}

function tofracnum(v)
	if v:find("%d[/^]%d") then return loadstring("return " .. v)() end
	return tonumber(v)
end

local ARG_OPTIONS = {
	{"-c",	"dmx",			false},
	{"-g",	"dmx",			true},
	{"-fb",	"fft_block",	tofracnum},
	{"-fc",	"fft_ch",		tonumber},
	{"-fo",	"fft_overlap",	tofracnum},
	{"-fw",	"fft_window",	string.upper},
	{"-p",	"preload",		tonumber},
	{"-s",	"stdin",		true},
	{"-m",	"mic",			tostring},
	{"-ms",	"mic_sr",		tonumber},
	{"-md",	"mic_dur",		tonumber},
	{"-r",	"loop",			tonumber},
}

-- -----------------------------------------------------------------------------
-- Global util funcs
-- -----------------------------------------------------------------------------
local ffi = require "ffi"
local bit = require "bit"

-- Python like formatter (format % var, or format % {var1, var2, ...})
getmetatable("").__mod = function(s, var)
	return type(var) == "table" and s:format(unpack(var)) or s:format(var)
end

-- Object accessor
function accessor(obj, get, set)
	return setmetatable(obj, {
		__index    = function(self, key)	return get[key]()  end,
		__newindex = function(self, key, v)		   set[key](v) end,
	})
end

ffi.cdef[[
	void *memmove(void*, const void*, size_t);

	typedef struct {
		long tv_sec;
		long tv_usec;
	} timeval;
	int gettimeofday(timeval*, void*);
	int poll(void*, int, int);
]]

-- Sub-second clock
function now()
  local t = ffi.new("timeval")
  if ffi.C.gettimeofday(t, nil) ~= 0 then return false end
  return t.tv_sec + t.tv_usec * 0.000001
end

-- Sub-second sleep
function msleep(ms)
  ffi.C.poll(nil, 0, ms)
end

-- Arg parser
function parseArg(arg, opt, opt_list)
	local i = 1
	while i <= #arg do 
		if arg[i]:sub(1, 1) == "-" then
			local j
			for k, v in ipairs(opt_list) do
				if arg[i] == v[1] then
					j = v[3]
					if type(j) == "function" then
						i = i + 1
						k, j = pcall(j, arg[i])
						if not k then return false end
					end
					opt[v[2]] = j
					break
				end
			end
			if j == nil then return false end
		else
			table.insert(opt.files, arg[i])
		end
		i = i + 1
	end
	return true
end


-- -----------------------------------------------------------------------------
-- FFT object
-- -----------------------------------------------------------------------------
MyFFT = (function () 
	-- create new FFT object  (block_size must be 2^n)
	local function createFFT(block_size, winfn)
		-- Create size related constant arrays
		local block_half = block_size / 2
		local tsin = ffi.new("double[?]", block_size + 1) -- Use ffi-double internally 
		local tcos = ffi.new("double[?]", block_size + 1) -- It's faster than lua table
		local twin = ffi.new("double[?]", block_size + 1) -- (about 25% on FFT)
		local trvs = ffi.new("double[?]", block_size + 1)
		
		-- Cache sin/cos/window-func tables
		for i = 1, block_size do
			tsin[i] = math.sin(-math.pi / i);
			tcos[i] = math.cos(-math.pi / i);
			twin[i] = winfn and winfn((i - 1) / (block_size - 1)) or 1
		end
		
		-- Cache reverse table
		local up = 1
		local dn = block_half
		trvs[1] = 1
		while up < block_size do
			for i = 1, up do
				trvs[up + i] = trvs[i] + dn
			end
			up = up * 2
			dn = dn / 2
		end
		
		-- Create recycle arrays
		-- The real/imag of ffi-double is the most effective change for performance
		local real = ffi.new("double[?]", block_size  + 1) -- {}
		local imag = ffi.new("double[?]", block_size  + 1) -- {}
		local spec = {} -- ffi.new("double[?]", block_half + 1) use standard table for public

		-- Perform a forward transform
		local function forward(buf, dB)
			-- init complex
			for i = 1, block_size do real[trvs[i]] = buf[i] * twin[i] end
			ffi.fill(imag, ffi.sizeof(imag))

			-- step calculation
			local up = 1
			while up < block_size do
				local up2 = up * 2
				local ps_real = tcos[up]
				local ps_imag = tsin[up]
				local cp_real = 1
				local cp_imag = 0

				for step = 1, up do
					local j = step
					while j <= block_size do
						local k = j + up
						local t_real = real[k]
						local t_imag = imag[k]
						t_real, t_imag  = cp_real * t_real - cp_imag * t_imag,
										  cp_real * t_imag + cp_imag * t_real
						real[k] = real[j] - t_real
						imag[k] = imag[j] - t_imag
						real[j] = real[j] + t_real
						imag[j] = imag[j] + t_imag
						j = j + up2
					end
					cp_real, cp_imag = cp_real * ps_real - cp_imag * ps_imag,
									   cp_real * ps_imag + cp_imag * ps_real
				end
				up = up2
			end
			
			-- spectrum/dB transform
			local invh = 1 / block_half
			local min  = dB and math.pow(10, dB / 20) 
			for i = 1, block_half do
				local t_real = real[i]
				local t_imag = imag[i]
				local val = math.sqrt(t_real * t_real + t_imag * t_imag) * invh
				spec[i] = dB and 20 * math.log10(math.max(min, val)) or val
			end
		end
		
		return accessor({
			-- method
			forward = forward,
		}, {
			-- getter
			block_size	= function() return block_size	end,
			spec_size	= function() return block_half	end,
			spec		= function() return spec		end,
		})
	end

	-- Constant values for window function
	local PI2 = math.pi * 2
	local PI4 = PI2 * 2

	-- public method
	return {
		new	= createFFT,
		
		-- Pre-defined window functions for createFFT
		RECT	= function(x) return 1 end,
		HANN	= function(x) return 0.5  - 0.5  * math.cos(PI2 * x) end,
		HAMMING	= function(x) return 0.54 - 0.46 * math.cos(PI2 * x) end,
		BLACKMAN= function(x) return 0.42 - 0.5  * math.cos(PI2 * x) + 0.08 * math.cos(PI4 * x) end,
		BARTLETT= function(x) return 1 - math.abs(2 * x - 1) end,
		SINE	= function(x) return math.sin(math.pi * x) end,
	}
end)()


-- -----------------------------------------------------------------------------
-- SoundBuffer object - Wave file parser
-- -----------------------------------------------------------------------------
MySoundBuffer = (function()
	-- WAV Magic
	local RIFF_HEADER	= 0x52494646 -- "RIFF"
	local WAVE_HEADER	= 0x57415645 -- "WAVE"
	local WAVE_CHUNK_FMT= 0x666D7420 -- "fmt "
	local WAVE_CHUNK_DAT= 0x64617461 -- "data"

	-- byte sign
	local SIGN1	= 128
	local SIGN2 = 32768
	local SIGN3 = 8388608
	local SIGN4 = 2147483648

	-- translate to a number from bin-string
	local function getVal(buf, pos, little, size)
		local last = pos + size - 1

		-- check endian
		if little < 0 then
			pos, last = last, pos
		end

		-- create value
		local ret = 0
		for i = pos, last, little do
			ret = ret * 0x100 + buf:byte(i)
		end
		return ret
	end

	-- two's complement
	local function signVal(val, max)
		return max > val and val or val - max * 2
	end

	-- simple get funcs
	local function getU8(buf, pos)		return 	       buf:byte(pos)					end
	local function getLU16(buf, pos)	return         getVal(buf, pos, -1, 2)			end
	local function getLS16(buf, pos)	return signVal(getVal(buf, pos, -1, 2), SIGN2)	end
	local function getLU32(buf, pos)	return         getVal(buf, pos, -1, 4)			end
	local function getLS32(buf, pos)	return signVal(getVal(buf, pos, -1, 4), SIGN4)	end
	local function getBU32(buf, pos)	return         getVal(buf, pos,  1, 4)			end

	-- normlize funcs for each bits
	local NORM_FUNCS = {
		function(buf, pos) return (getU8(buf, pos) - SIGN1)       	/ SIGN1	end, -- 8bit:  unsigned(0..255)
		function(buf, pos) return getLS16(buf, pos)            		/ SIGN2	end, -- 16bit: signed  (-32768..32767)
		function(buf, pos) return bit.arshift(getLS32(buf, pos), 8)	/ SIGN3	end, -- 24bit: signed  (-8388608..8388607)
	}
	
	-- read and normalize for specified size from file
	local function readBuffer(srcbuf, file, readsize, offset)
		local data = file:read(readsize)
		if #data ~= readsize then error "Data too short!" end

		-- put normalized data to srcbuf of each channel
		local b8	= srcbuf.byte
		local norm	= NORM_FUNCS[b8]
		local pos	= 1
		for i = 1, readsize / srcbuf.ch / b8 do
			for j, v in ipairs(srcbuf) do
				v[i + offset] = norm(data, pos)
				pos = pos + b8
			end
		end
	end

	-- Create merge buffer
	local function getMerge(srcbuf)
		-- Use cache if exist
		if srcbuf[0] then return srcbuf[0] end

		-- If buffera has only ch-1, merge is same as 1ch
		local ch = srcbuf.ch
		if ch == 1 then return srcbuf[1] end
		
		-- Creates a merged buffer
		local merge = {}
		for i, v in ipairs(srcbuf[1]) do
			for j = 2, ch do
				v = v + srcbuf[j][i]
			end
			merge[i] = v / ch
		end
		srcbuf[0] = merge -- Add buffer to [0] for next operation
		return merge
	end

	-- ReadFn for preload buffer (for small wave file)
	local function getPreloadBuffer(srcbuf, ch, pos, size)
		-- copy preload buffer to return buffer
		local src = ch == 0 and getMerge(srcbuf) or srcbuf[ch]
		local dst = srcbuf.signal
		for i = 1, size do dst[i] = src[i + pos] end
		return dst -- Skip trunc by size even if dst was shrinked
	end

	-- ReadFn for on-demand buffer (for big wave file)
	local function getOnDemandBuffer(srcbuf, ch, pos, size)
		local prog = pos - srcbuf.cur_pos
		if prog < 0 then error "OnDemandBuffer: Rewind : not support" end -- for popen
						
		-- Check reusable area and read left data from file
		local reuse = srcbuf.cur_len - prog
		if reuse < 0 then
			-- Skip useless data
			srcbuf.file:seek("cur", reuse * srcbuf.byte * srcbuf.ch)
			reuse = 0
		elseif reuse > 0 and prog > 0 then
			-- Shift reuse data of each ch (should be used a ring buffer...?)
			for j, v in ipairs(srcbuf) do
				for i = 1, reuse do v[i] = v[i + prog] end
			end
			srcbuf.cur_len = reuse
		end
		local left = size - reuse
		if left > 0 then
			readBuffer(srcbuf, srcbuf.file, left * srcbuf.byte * srcbuf.ch, reuse)
			srcbuf.cur_len = size
		end
		srcbuf.cur_pos = pos
				
		-- copy read buffer to return buffer
		local dst = srcbuf.signal
		if ch > 0 then
			local src = srcbuf[ch]
			for i = 1, size do dst[i] = src[i] end
		else -- Merge
			local src_ch = srcbuf.ch
			for i = 1, size do
				local val = 0
				for j = 1, src_ch do
					val = val + srcbuf[j][i]
				end
				dst[i] = val / src_ch
			end
		end

		return dst -- skip trunc by size even if dst was shrinked
	end

	local function dtor(srcbuf)
		if srcbuf.file then
			srcbuf.file:close()
			srcbuf.file = nil
		end
	end

	-- create and return a common public methods for SoundBuffer  
	local function commonAccessor(srcbuf, readFn)
		return accessor({
			-- method
			getSignal	= function(ch, pos, size) return readFn(srcbuf, ch, pos, size) end,
			close		= function() return dtor(srcbuf) end
		}, {
			-- getter
			ch	= function() return srcbuf.ch				end,
			sr	= function() return srcbuf.sr				end,
			bits= function() return srcbuf.bit	 			end,
			len	= function() return srcbuf.len				end,
			dur	= function() return srcbuf.len / srcbuf.sr	end
		})
	end

	local function createBuffer(file, preload)
		-- Check WAV header
		local data = file:read(12)
		if not data or #data ~= 12 or getBU32(data, 1) ~= RIFF_HEADER or getBU32(data, 9) ~= WAVE_HEADER then return false, "Invalid wave format " end
	
		-- Check all chunks
		local srcbuf = {format = 0}
		while true do
			data = file:read(8)
			if #data ~= 8 then 														return false, "No data section"	end -- EOF
		
			local chunk = getBU32(data, 1)
			local size  = getLU32(data, 5)

			if chunk == WAVE_CHUNK_FMT then -- Format chunk ------------
				data = file:read(size)
				if #data ~= size then												return false, "WAV file is too short." end

				-- Check format ID (currently support PCM:1 only)
				srcbuf.format = getLU16(data, 1)
				if srcbuf.format ~= 1 then 											return false, "Invalid WAV format (1:PCM only): " .. srcbuf.format end
				
				-- Check number of channels (>= 1)
				srcbuf.ch = getLU16(data, 3)
				if srcbuf.ch < 1 then												return false, "Invalid channel number (>= 1ch): " .. srcbuf.ch end

				-- Check sampling rate (>= 1kHz)
				srcbuf.sr = getLU32(data, 5)
				if srcbuf.sr < 1000 then											return false, "Invalid sampling rate (>= 1kHz): " .. srcbuf.sr end

				-- Check bit depth (max.24bit)
				srcbuf.bit  = getLU16(data, 15)
				srcbuf.byte = srcbuf.bit / 8
				if srcbuf.bit ~= 8 and srcbuf.bit ~= 16 and srcbuf.bit ~= 24 then	return false, "Invalid bit depth (8, 16, 24): " .. srcbuf.bit end

			elseif chunk == WAVE_CHUNK_DAT then -- Data chunk -------------
			 	-- FMT chunk should be exist before DAT chunk
				if srcbuf.format ~= 1 then											return false, "Invalid WAV format." end

				-- Create empty buffer arrays for all channels
				srcbuf.len 	  = size / srcbuf.ch / srcbuf.byte
				srcbuf.signal = {} -- result buffer (for recyle)
				for i = 1, srcbuf.ch do
					srcbuf[i] = {} -- read buffer for each ch
				end

				-- Check wave file size to decide which buffer-mode uses
				if size > preload then
					-- Too big, use the on-demand buffer reader (for Microphone stream, etc)
					srcbuf.cur_pos = 1
					srcbuf.cur_len = 0
					srcbuf.file    = file
					return commonAccessor(srcbuf, getOnDemandBuffer)
				else
					-- Wave size is small, then read at once and return preload buffer reader
					readBuffer(srcbuf, file, size, 0) -- read all at once
					file:close()
					return commonAccessor(srcbuf, getPreloadBuffer)
				end
			end
		end
	end

	-- Load wave file
	local function loadWavFile(fname, preload)
		local file,err = io.open(fname, "rb")
		if not file then return false, err end
		return createBuffer(file, preload)
	end

	local function openMic(dev, sr, dur, bit)
		local mic, err = io.popen("/usr/bin/arecord -D " .. dev .. " -r " .. sr .. " -f " .. bit .. " -c 1 -t wav -d " .. dur, "r")
		if not mic then return false, err end
		return createBuffer(mic, 0)
	end
	
	-- public method
	return {
		loadWavFile = loadWavFile,
		openMic		= openMic,
		readStdin	= function() return createBuffer(io.stdin, 0) end,
	}
end)()

-- -----------------------------------------------------------------------------
-- Parameter settings
-- -----------------------------------------------------------------------------
if not parseArg(arg, options, ARG_OPTIONS) then
	print(USAGE)
	return
end

-- Create FFT object by specified parameter
local fft = MyFFT.new(options.fft_block, MyFFT[options.fft_window])

-- -----------------------------------------------------------------------------
-- Display settings
-- -----------------------------------------------------------------------------
-- CUI parameter (require terminal columns >= 80)
local CUI_SIGWIDTH = 15
local CUI_FFTWIDTH = 55
local CUI_SPC	= " "
local CUI_FILL	= "="
local CUI_GREY	= " .-+*$@#"
function showCuiLine(buf, pos, fft_spec, sbuf)
	-- show position time
	io.write("%7.3f" % {pos / sbuf.sr})

	-- Show signal waveform
	local min = buf[1]
	local max = min
	for i = 2, options.fft_block do
		val = buf[i]
		if min > val then
		    min = val
		elseif max < val then
		    max = val
		end
	end
	min = math.floor((min + 1) * CUI_SIGWIDTH / 2)
	max = math.ceil ((max + 1) * CUI_SIGWIDTH / 2)
	io.write(CUI_SPC:rep(min), CUI_FILL:rep(max - min), CUI_SPC:rep(CUI_SIGWIDTH - max), "|")

	-- Show FFT flow
	local mindB = options.min_dB
	for i = 1, CUI_FFTWIDTH do
		local val = fft_spec[math.ceil(i * fft.spec_size / CUI_FFTWIDTH)]
		local col = math.floor((1 - val / mindB) * CUI_GREY:len()) + 1 -- for dB sepc
		io.write(CUI_GREY:sub(col, col))
	end
	io.write("|\n")
end

-- display paramter and functions for DMX
local DMX
if options.dmx then DMX = require "DisplayManX" end

function FillRect(pbuff, x,  y,  w,  h, val)
    local dataPtr = ffi.cast("uint8_t *", pbuff.Data);

    local row;
    local col;
    for row=y, y+h-1  do
    	local rowPtr = ffi.cast("int16_t *", (dataPtr + (pbuff.Pitch*row)));
        for col=x, x+w-1 do
            rowPtr[col] = val;
        end
    end
end

-- Color converter for RGB565
function RGB(r,g,b)
    return math.floor(r / 8) * 0x0800 + 
           math.floor(g / 4) * 0x0020 + 
           math.floor(b / 8) 
end

-- Color palette for flow area
local pal = {[0]=0}
for i = 0,0x1f do pal[#pal + 1] = i * 0x800         end -- black to red
for i = 1,0x1f do pal[#pal + 1] = i * 0x40 + 0xf800 end -- red to yellow
for i = 1,0x1f do pal[#pal + 1] = i        + 0xffe0 end -- yellow to white

local Display, dwidth, dheight, pbuff, dataPtr, mainView
local dmergin = 16
function showDmxLine(buf, pos, fft_spec)
	if not Display then
		Display = DMXDisplay()
		local info = Display:GetInfo()
		print(string.format("Display is %d x %d", info.width, info.height))
		dwidth = info.width  - dmergin * 2
		dheight= info.height - dmergin * 2
		pbuff = DMX.DMXPixelData(dwidth, dheight);
		dataPtr = ffi.cast("uint8_t *", pbuff.Data)
		FillRect(pbuff, 0, 0, dwidth, dheight, RGB(32, 32, 32))
		mainView = Display:CreateView(dwidth, dheight, dmergin, dmergin, nil, nil, 0.9)
	end

	-- Calc params for next line --
	local wave_height = math.floor(dheight * 0.1)
	local spec_width  = math.floor(dwidth  * 0.8)
	local sign_width  = dwidth - spec_width - 4
	local rowPtr  = ffi.cast("uint16_t *", dataPtr + pbuff.Pitch * wave_height)
	
	-- flow
	--ffi.copy(dataPtr, dataPtr + pbuff.Pitch, pbuff.Pitch * (dheight - 1)) -- upper scroll
	ffi.C.memmove(dataPtr + pbuff.Pitch * (wave_height + 1), dataPtr + pbuff.Pitch * wave_height, pbuff.Pitch * (dheight - wave_height - 1)) -- fall scroll
	local wave_cache = {}
	local mindB = options.min_dB
	for i = 1, spec_width do
		local val = fft_spec[math.ceil(i * fft.spec_size / spec_width)]
		val = 1 - val / mindB  -- for dB sepc
		rowPtr[i - 1] = pal[math.floor(val * #pal)]
		
		wave_cache[i] = (1 - val) * wave_height
	end

	-- border
	rowPtr[spec_width + 0] = RGB(128, 128, 128)
	rowPtr[spec_width + 1] = RGB(255, 255, 255)
	rowPtr[spec_width + 2] = RGB( 64,  64,  64)

	-- signal
	local min = buf[1]
	local max = min
	for i = 2, options.fft_block do
		val = buf[i]
		if min > val then
		    min = val
		elseif max < val then
		    max = val
		end
	end
	min = math.floor((min + 1) * sign_width / 2)
	max = math.ceil ((max + 1) * sign_width / 2)
	local base = spec_width + 4
	for i = 1,       min - 1    do rowPtr[base + i] = 0x0008 end
	for i = min,     max        do rowPtr[base + i] = 0x841F end
	for i = max + 1, sign_width do rowPtr[base + i] = 0x0008 end
	
	-- wave
	for y = 1, wave_height - 2 do
		rowPtr = ffi.cast("int16_t *", dataPtr + pbuff.Pitch * y)
		for x = 1, spec_width do
			rowPtr[x - 1] = wave_cache[x] < y and 0x07E0 or 0x0100
		end
	end

	mainView:CopyPixelBuffer(pbuff, 0, 0, dwidth, dheight)
end

-- -----------------------------------------------------------------------------
-- Main
-- -----------------------------------------------------------------------------
local loop_count = 0
while loop_count < options.loop do
	loop_count = loop_count + 1
	-- Load Wave file
	local t_tick = now()
	local sbuf, err
	if options.stdin then
		sbuf, err = MySoundBuffer.readStdin()
		options.loop = 1 -- force one time
	elseif #options.files > 0 then
		sbuf, err = MySoundBuffer.loadWavFile(options.files[loop_count % #options.files + 1], options.preload)
	else
		sbuf, err = MySoundBuffer.openMic(options.mic, options.mic_sr, options.mic_dur, options.mic_bit)
	end
	
	if not sbuf then
		print("Parse failed:", err)
		return
	end
	print("Parsed: %dch %dkHz %dbit %dsamp=%.3fs (parse%.3fms)" % {sbuf.ch, sbuf.sr / 1000, sbuf.bits, sbuf.len, sbuf.dur, now() - t_tick})

	local pos = 1
	local step = math.max(1, math.floor(options.fft_block * (1 - options.fft_overlap)))
	local mindB = options.min_dB
	local cputime = os.clock()
	t_tick = now()
	while pos + options.fft_block <= sbuf.len do
		-- Perform FFT	
		local buf = sbuf.getSignal(options.fft_ch, pos, options.fft_block)
		fft.forward(buf, mindB)
		-- Show result
		if options.dmx then
			showDmxLine(buf, pos, fft.spec, sbuf)
		else
			showCuiLine(buf, pos, fft.spec, sbuf)
		end
	
		-- Set next block position overlapped by specified ratio
		pos = pos + step

		-- Timing control
		local wait = pos / sbuf.sr - (now() - t_tick)
		if wait > 0 then msleep(wait * 1000) end
	end 
	
	-- Show CPU load
	t_tick = now() - t_tick
	cputime = os.clock() - cputime
	print("End CPU LOAD %.1f%%" % {cputime * 100 / t_tick})

	sbuf.close()
end
