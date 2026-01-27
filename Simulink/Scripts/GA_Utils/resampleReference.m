function I_ref = resampleReference(RefTS, t_query)
% resampleReference: 타임시리즈 형태의 레퍼런스를 특정 시간 벡터로 리샘플링
    t_query = t_query(:);
    resampled = resample(RefTS, t_query);
    I_ref = squeeze(resampled.Data);
    I_ref = I_ref(:);
end
