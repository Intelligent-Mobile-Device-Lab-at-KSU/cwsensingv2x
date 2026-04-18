classdef idx_helpers
% IDX_HELPERS  Static utility methods for resource index conversions
%
% The RH matrix is Nslch x N (column-major in MATLAB):
%   Linear index = subchannel + (subframe - 1) * Nslch
%
% All methods support vector inputs for efficient batch operations.
%
% Usage:
%   sf  = idx_helpers.to_subframe(linearIdx, Nslch)
%   sc  = idx_helpers.to_subchannel(linearIdx, Nslch)
%   idx = idx_helpers.to_linear(subchannel, subframe, Nslch)
%   rc  = idx_helpers.draw_reselection_counter(RRI_ms)
%   L   = idx_helpers.compute_L_subCH(msg_bytes, Nrbs, bits_per_prb)
%
% References:
%   3GPP TS 36.321 Section 5.14.1.1 (SL_RESOURCE_RESELECTION_COUNTER)
%   3GPP TS 36.213 Table 7.1.7.2.1-1 (TBS determination)

    methods (Static)

        function sf = to_subframe(idx, Nslch)
            % TO_SUBFRAME  Convert linear RH index to subframe number (1-based)
            %   sf = idx_helpers.to_subframe(idx, Nslch)
            %   Supports vector input (element-wise).
            sf = floor((idx - 1) ./ Nslch) + 1;
        end

        function sc = to_subchannel(idx, Nslch)
            % TO_SUBCHANNEL  Convert linear RH index to subchannel number (1-based)
            %   sc = idx_helpers.to_subchannel(idx, Nslch)
            %   Supports vector input (element-wise).
            sc = mod(idx - 1, Nslch) + 1;
        end

        function idx = to_linear(subchannel, subframe, Nslch)
            % TO_LINEAR  Convert (subchannel, subframe) to linear RH index
            %   idx = idx_helpers.to_linear(subchannel, subframe, Nslch)
            %   Supports vector inputs (element-wise).
            idx = subchannel + (subframe - 1) .* Nslch;
        end

        function rc = draw_reselection_counter(RRI_ms)
            % DRAW_RESELECTION_COUNTER  Draw SL_RESOURCE_RESELECTION_COUNTER
            %   rc = idx_helpers.draw_reselection_counter(RRI_ms)
            %
            %   Per 3GPP TS 36.321 Section 5.14.1.1:
            %     RRI <= 100ms:          uniformly from [5, 15]
            %     RRI  = 200ms:          uniformly from [10, 30]
            %     RRI in {300..1000ms}:  uniformly from [25, 75]
            if RRI_ms <= 100
                rc = randi([5, 15]);
            elseif RRI_ms == 200
                rc = randi([10, 30]);
            else
                rc = randi([25, 75]);
            end
        end

        function L = compute_L_subCH(msg_bytes, Nrbs, bits_per_prb)
            % COMPUTE_L_SUBCH  Number of contiguous subchannels needed for a message
            %   L = idx_helpers.compute_L_subCH(msg_bytes, Nrbs, bits_per_prb)
            %
            %   Computes the minimum number of contiguous subchannels (L_subCH)
            %   required to carry a message of msg_bytes, given:
            %     Nrbs         - PRBs per subchannel
            %     bits_per_prb - Approximate bits per PRB at the chosen MCS
            %
            %   Accounts for PSCCH (SCI) overhead of 2 PRBs.
            %   Based on simplified TBS from 3GPP TS 36.213 Table 7.1.7.2.1-1.
            %
            %   Typical bits_per_prb values:
            %     MCS 5 (QPSK,  low rate):  ~180
            %     MCS 7 (QPSK, mid rate):   ~260   <-- common V2X default
            %     MCS 11 (16QAM, mid rate): ~420
            %     MCS 14 (16QAM, high rate): ~550

            SCI_prbs = 2;  % PSCCH overhead
            msg_bits = msg_bytes * 8;

            for L = 1:100
                data_prbs = L * Nrbs - SCI_prbs;
                if data_prbs < 1, continue; end
                tbs = data_prbs * bits_per_prb;
                if tbs >= msg_bits
                    return;
                end
            end
            % Should never reach here for realistic message sizes
            L = 100;
        end

    end
end
