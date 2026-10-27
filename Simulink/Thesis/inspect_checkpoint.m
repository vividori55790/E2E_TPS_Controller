function inspect_checkpoint(matFile, outCsv)
% inspect_checkpoint.m -- read-only look at a Generate_TPS_Dataset checkpoint (MAT v7.3) and export its arrays.
%   matlab -batch "cd('Simulink/Thesis'); inspect_checkpoint"
% Prints every variable with its size/class and the valid / completed / error counts, then writes all
% per-row arrays (V2, D0, D1, D2, I_pk, P_act, P_act_HV, valid) to a CSV for comparison in Python.
    thesisDir = fileparts(mfilename('fullpath'));
    if nargin < 1 || isempty(matFile)
        matFile = fullfile(fileparts(thesisDir), 'Data', 'TPS_Checkpoint_done_20260316_021229.mat');
    end
    if nargin < 2 || isempty(outCsv)
        outCsv = fullfile(thesisDir, 'out', 'checkpoint_20260316_021229.csv');
    end
    fprintf('file: %s\n', matFile);
    info = whos('-file', matFile);
    for k = 1:numel(info)
        fprintf('  %-20s %-10s %s\n', info(k).name, info(k).class, mat2str(info(k).size));
    end
    S = load(matFile);
    if isfield(S, 'dataset_valid'),  fprintf('valid rows      : %d of %d\n', nnz(S.dataset_valid), numel(S.dataset_valid)); end
    if isfield(S, 'completed_sims'), fprintf('completed sims  : %d of %d\n', nnz(S.completed_sims), numel(S.completed_sims)); end
    if isfield(S, 'error_msg')
        nerr = sum(~cellfun(@isempty, S.error_msg));
        fprintf('sims with error : %d\n', nerr);
    end
    names = {'dataset_V2', 'dataset_D0', 'dataset_D1', 'dataset_D2', 'dataset_I_pk', 'dataset_P_act', 'dataset_P_act_HV', 'dataset_valid'};
    have = names(isfield(S, names));
    cols = cell(1, numel(have));
    for k = 1:numel(have)
        cols{k} = double(S.(have{k})(:));
    end
    T = table(cols{:}, 'VariableNames', strrep(have, 'dataset_', ''));
    writetable(T, outCsv);
    fprintf('wrote %s (%d rows, columns: %s)\n', outCsv, height(T), strjoin(T.Properties.VariableNames, ', '));
end
