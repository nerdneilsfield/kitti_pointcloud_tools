package io.github.nerdneilsfield.kpt.viewer

import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.FlowRow
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.ExperimentalLayoutApi
import androidx.compose.material3.Button
import androidx.compose.material3.CircularProgressIndicator
import androidx.compose.material3.FilterChip
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Slider
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.material3.TopAppBar
import androidx.compose.material3.ExperimentalMaterial3Api
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.LifecycleEventObserver
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import androidx.lifecycle.viewmodel.compose.viewModel

class MainActivity : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContent {
            MaterialTheme {
                val model: ViewerViewModel = viewModel()
                val picker = rememberLauncherForActivityResult(
                    ActivityResultContracts.OpenDocument(),
                ) { uri ->
                    uri?.let(model::open)
                }
                ViewerScreen(
                    model = model,
                    onOpen = { picker.launch(arrayOf("application/octet-stream", "text/plain", "*/*")) },
                )
            }
        }
    }
}

@OptIn(ExperimentalMaterial3Api::class)
@Composable
private fun ViewerScreen(
    model: ViewerViewModel,
    onOpen: () -> Unit,
) {
    val state by model.uiState.collectAsStateWithLifecycle()
    val colorMode by model.colorMode.collectAsStateWithLifecycle()
    val pointSize by model.pointSize.collectAsStateWithLifecycle()

    Scaffold(
        topBar = {
            TopAppBar(
                title = { Text("KPT Point Cloud Viewer") },
                actions = { TextButton(onClick = onOpen) { Text("Open") } },
            )
        },
    ) { padding ->
        Box(
            modifier = Modifier
                .fillMaxSize()
                .padding(padding),
        ) {
            when (val current = state) {
                ViewerUiState.Empty -> EmptyState(onOpen)
                is ViewerUiState.Loading -> LoadingState(model::cancelLoad)
                is ViewerUiState.Error -> ErrorState(current, onOpen)
                is ViewerUiState.Ready -> ReadyViewer(
                    cloud = current.cloud,
                    colorMode = colorMode,
                    pointSize = pointSize,
                    onColorMode = model::setColorMode,
                    onPointSize = model::setPointSize,
                )
            }
        }
    }
}

@Composable
private fun EmptyState(onOpen: () -> Unit) {
    Column(
        modifier = Modifier.fillMaxSize().padding(32.dp),
        horizontalAlignment = Alignment.CenterHorizontally,
        verticalArrangement = Arrangement.Center,
    ) {
        Text("Open one KITTI BIN, PCD, or PLY point cloud.")
        Spacer(Modifier.height(16.dp))
        Button(onClick = onOpen) { Text("Choose file") }
    }
}

@Composable
private fun LoadingState(onCancel: () -> Unit) {
    Column(
        modifier = Modifier.fillMaxSize(),
        horizontalAlignment = Alignment.CenterHorizontally,
        verticalArrangement = Arrangement.Center,
    ) {
        CircularProgressIndicator()
        Spacer(Modifier.height(12.dp))
        Text("Decoding point cloud…")
        TextButton(onClick = onCancel) { Text("Cancel") }
    }
}

@Composable
private fun ErrorState(state: ViewerUiState.Error, onOpen: () -> Unit) {
    Column(
        modifier = Modifier.fillMaxSize().padding(32.dp),
        horizontalAlignment = Alignment.CenterHorizontally,
        verticalArrangement = Arrangement.Center,
    ) {
        Text(state.code.name, color = MaterialTheme.colorScheme.error)
        Spacer(Modifier.height(8.dp))
        Text(state.message)
        Spacer(Modifier.height(16.dp))
        Button(onClick = onOpen) { Text("Choose another file") }
    }
}

@Composable
private fun ReadyViewer(
    cloud: NativeCloud,
    colorMode: ColorMode,
    pointSize: Float,
    onColorMode: (ColorMode) -> Unit,
    onPointSize: (Float) -> Unit,
) {
    var surface by remember { mutableStateOf<GlesPointCloudView?>(null) }
    val lifecycle = androidx.lifecycle.compose.LocalLifecycleOwner.current.lifecycle
    DisposableEffect(lifecycle) {
        val observer = LifecycleEventObserver { _, event ->
            when (event) {
                Lifecycle.Event.ON_RESUME -> surface?.onResume()
                Lifecycle.Event.ON_PAUSE -> surface?.onPause()
                else -> Unit
            }
        }
        lifecycle.addObserver(observer)
        onDispose {
            lifecycle.removeObserver(observer)
            surface?.close()
            surface = null
        }
    }

    Column(Modifier.fillMaxSize()) {
        AndroidView(
            modifier = Modifier.fillMaxWidth().weight(1.0f),
            factory = { context ->
                GlesPointCloudView(context).also { view ->
                    surface = view
                    view.setCloud(cloud)
                }
            },
            update = { view ->
                view.setCloud(cloud)
                view.setColorMode(colorMode)
                view.setPointSize(pointSize)
            },
        )
        ViewerControls(
            metadata = cloud.metadata,
            colorMode = colorMode,
            pointSize = pointSize,
            onColorMode = onColorMode,
            onPointSize = onPointSize,
            onFit = { surface?.fit() },
        )
    }
}

@OptIn(ExperimentalLayoutApi::class)
@Composable
private fun ViewerControls(
    metadata: CloudMetadata,
    colorMode: ColorMode,
    pointSize: Float,
    onColorMode: (ColorMode) -> Unit,
    onPointSize: (Float) -> Unit,
    onFit: () -> Unit,
) {
    Column(
        modifier = Modifier
            .fillMaxWidth()
            .background(Color(0xFFF5F6FC))
            .padding(horizontal = 12.dp, vertical = 8.dp),
    ) {
        Row(
            modifier = Modifier.fillMaxWidth(),
            verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.SpaceBetween,
        ) {
            Text("${metadata.pointCount} points")
            TextButton(onClick = onFit) { Text("Fit") }
        }
        FlowRow(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            ColorMode.entries.forEach { mode ->
                val enabled = when (mode) {
                    ColorMode.RGB -> metadata.hasColor
                    ColorMode.INTENSITY -> metadata.hasIntensity
                    else -> true
                }
                FilterChip(
                    selected = colorMode == mode,
                    onClick = { onColorMode(mode) },
                    enabled = enabled,
                    label = { Text(mode.name) },
                )
            }
        }
        Row(verticalAlignment = Alignment.CenterVertically) {
            Text("Point ${"%.1f".format(pointSize)}")
            Slider(
                value = pointSize,
                onValueChange = onPointSize,
                valueRange = 1.0f..12.0f,
                modifier = Modifier.weight(1.0f).padding(start = 12.dp),
            )
        }
    }
}
