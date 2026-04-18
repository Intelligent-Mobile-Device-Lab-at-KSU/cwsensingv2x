function write_complex_binary(filename, data, mode)
% WRITE_COMPLEX_BINARY  Write complex samples to GNU Radio binary format.
%
%   write_complex_binary(filename, data)        - write (overwrite)
%   write_complex_binary(filename, data, 'a')   - append
%
%   Writes interleaved float32 I/Q pairs: [I0 Q0 I1 Q1 ...]
%   Compatible with read_complex_binary() used by GNU Radio and mDopPx.
%
%   Inputs:
%     filename - Output file path (.dat or .bin)
%     data     - Complex column or row vector (double or single)
%     mode     - 'w' (default, overwrite) or 'a' (append)

    if nargin < 3
        mode = 'w';
    end

    % Validate mode
    if ~ismember(mode, {'w', 'a'})
        error('write_complex_binary:invalidMode', ...
              'mode must be ''w'' (overwrite) or ''a'' (append). Got ''%s''.', mode);
    end

    % Force column vector
    data = data(:);

    % Interleave real and imaginary parts: [I0 Q0 I1 Q1 ...]
    interleaved = zeros(2 * length(data), 1, 'single');
    interleaved(1:2:end) = single(real(data));
    interleaved(2:2:end) = single(imag(data));

    % Open file
    if strcmp(mode, 'w')
        fid = fopen(filename, 'w', 'ieee-le');
    else
        fid = fopen(filename, 'a', 'ieee-le');
    end

    if fid < 0
        error('write_complex_binary:fileError', ...
              'Could not open file: %s', filename);
    end

    % Write interleaved float32
    fwrite(fid, interleaved, 'float32');
    fclose(fid);

end
