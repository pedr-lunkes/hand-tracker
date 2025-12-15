import pandas
import numpy as np
from scipy.stats import skew, kurtosis


class Extrator:
    COLUNAS_SENSORES = ['qx', 'qy', 'qz', 'qw', 'px', 'py', 'pz']

    def extrair_features(self, df):
        dados_features = {}
        
        for col in self.COLUNAS_SENSORES:
            series = df[col].values
            
            dados_features[f'{col}_mean'] = np.mean(series)
            dados_features[f'{col}_std'] = np.std(series)
            dados_features[f'{col}_max'] = np.max(series)
            dados_features[f'{col}_min'] = np.min(series)
            dados_features[f'{col}_skew'] = skew(series)
            dados_features[f'{col}_kurt'] = kurtosis(series)
            
            # fft pq tomei taca no projeto de algelin :(

            sinal_cent = series - np.mean(series)
            magnitude = np.abs(np.fft.rfft(sinal_cent))
            
            if len(magnitude) > 0:
                dados_features[f'{col}_fft_mean'] = np.mean(magnitude)
                dados_features[f'{col}_fft_max'] = np.max(magnitude)
                dados_features[f'{col}_fft_std'] = np.std(magnitude)

        return dados_features
