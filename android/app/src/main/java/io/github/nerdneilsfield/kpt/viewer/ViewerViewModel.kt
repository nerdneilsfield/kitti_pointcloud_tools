package io.github.nerdneilsfield.kpt.viewer

import android.app.Application
import android.net.Uri
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import java.util.concurrent.atomic.AtomicLong
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.Job
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch

sealed interface ViewerUiState {
    data object Empty : ViewerUiState
    data class Loading(val uri: Uri) : ViewerUiState
    data class Ready(val uri: Uri, val cloud: NativeCloud) : ViewerUiState
    data class Error(val uri: Uri?, val code: DecodeErrorCode, val message: String) : ViewerUiState
}

enum class ColorMode { RGB, INTENSITY, Z, SOLID }

class ViewerViewModel(application: Application) : AndroidViewModel(application) {
    private val repository = PointCloudRepository(application)
    private val generation = AtomicLong(0L)
    private var decodeJob: Job? = null
    private val _uiState = MutableStateFlow<ViewerUiState>(ViewerUiState.Empty)
    val uiState: StateFlow<ViewerUiState> = _uiState.asStateFlow()
    private val _colorMode = MutableStateFlow(ColorMode.INTENSITY)
    val colorMode: StateFlow<ColorMode> = _colorMode.asStateFlow()
    private val _pointSize = MutableStateFlow(3.0f)
    val pointSize: StateFlow<Float> = _pointSize.asStateFlow()

    fun open(uri: Uri) {
        val requestGeneration = generation.incrementAndGet()
        decodeJob?.cancel()
        releaseReadyCloud()
        _uiState.value = ViewerUiState.Loading(uri)
        decodeJob = viewModelScope.launch {
            try {
                val cloud = repository.decode(uri)
                if (generation.get() != requestGeneration) {
                    cloud.close()
                    return@launch
                }
                _colorMode.value = when {
                    cloud.metadata.hasIntensity -> ColorMode.INTENSITY
                    cloud.metadata.hasColor -> ColorMode.RGB
                    else -> ColorMode.SOLID
                }
                _uiState.value = ViewerUiState.Ready(uri, cloud)
            } catch (cancelled: CancellationException) {
                throw cancelled
            } catch (error: NativeDecodeException) {
                if (generation.get() == requestGeneration) {
                    _uiState.value = ViewerUiState.Error(uri, error.code, error.message.orEmpty())
                }
            } catch (error: Throwable) {
                if (generation.get() == requestGeneration) {
                    _uiState.value = ViewerUiState.Error(
                        uri,
                        DecodeErrorCode.INTERNAL,
                        error.message ?: "unexpected viewer failure",
                    )
                }
            }
        }
    }

    fun cancelLoad() {
        generation.incrementAndGet()
        decodeJob?.cancel()
        decodeJob = null
        _uiState.value = ViewerUiState.Empty
    }

    fun setColorMode(mode: ColorMode) {
        _colorMode.value = mode
    }

    fun setPointSize(size: Float) {
        _pointSize.value = size.coerceIn(1.0f, 12.0f)
    }

    private fun releaseReadyCloud() {
        (_uiState.value as? ViewerUiState.Ready)?.cloud?.close()
    }

    override fun onCleared() {
        generation.incrementAndGet()
        decodeJob?.cancel()
        releaseReadyCloud()
        repository.close()
        super.onCleared()
    }
}
