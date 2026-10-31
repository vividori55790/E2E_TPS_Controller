function f1_probe_scope()
% f1_probe_scope.m -- (2026-10-31) what does a Scope block of the dataset-era model accept for logging? Read only; the model is closed without saving.
    here = fileparts(mfilename('fullpath'));
    era = fullfile(here, 'dataset_era');
    addpath(fullfile(fileparts(here), 'Modules')); addpath(era);
    Simulink.fileGenControl('set', 'CacheFolder', fullfile(here, 'out', 'f1_cache'));
    load_system(fullfile(era, 'Control_Logic_dataset_era.slx'));
    m = 'Control_Logic_dataset_era';
    for b = {'Scope2', 'Scope13'}
        blk = [m '/' b{1}];
        fprintf('%s BlockType %s Commented %s\n', blk, get_param(blk, 'BlockType'), get_param(blk, 'Commented'));
        for p = {'DataLogging', 'DataLoggingVariableName', 'DataLoggingSaveFormat', 'DataLoggingLimitDataPoints', 'DataLoggingDecimateData', 'ScopeSpecificationString', 'SaveToWorkspace', 'SaveName', 'DataFormat', 'LimitDataPoints'}
            try
                fprintf('  %s = %s\n', p{1}, char(string(get_param(blk, p{1}))));
            catch ME
                fprintf('  %s: not a parameter (%s)\n', p{1}, ME.identifier);
            end
        end
        for v = {'Dataset', 'Array', 'Structure', 'StructureWithTime', 'Structure With Time', 'Timeseries'}
            try
                set_param(blk, 'DataLoggingSaveFormat', v{1});
                fprintf('  set_param format %s: ok -> %s\n', v{1}, get_param(blk, 'DataLoggingSaveFormat'));
            catch ME
                fprintf('  set_param format %s: %s\n', v{1}, ME.message);
            end
        end
    end
    close_system(m, 0);
end
